/* -*- Mode: C; c-basic-offset: 4; indent-tabs-mode: nil -*- */
/*
   Copyright (C) 2010 Red Hat, Inc.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappbuffer.h>
#include <gst/app/gstappsink.h>

#include "spice-gstaudio.h"
#include "spice-common.h"
#include "spice-session.h"

#define SPICE_GSTAUDIO_GET_PRIVATE(obj)                                  \
    (G_TYPE_INSTANCE_GET_PRIVATE((obj), SPICE_TYPE_GSTAUDIO, spice_gstaudio))

G_DEFINE_TYPE(SpiceGstAudio, spice_gstaudio, SPICE_TYPE_AUDIO)

struct stream {
    GstElement              *pipe;
    GstElement              *src;
    GstElement              *sink;
    guint                   rate;
    guint                   channels;
};

struct spice_gstaudio {
    SpiceSession            *session;
    SpiceChannel            *pchannel;
    SpiceChannel            *rchannel;
    struct stream           playback;
    struct stream           record;
    guint                   mmtime_id;
};

static void channel_event(SpiceChannel *channel, SpiceChannelEvent event,
                          gpointer data);

static void spice_gstaudio_finalize(GObject *obj)
{
    G_OBJECT_CLASS(spice_gstaudio_parent_class)->finalize(obj);
}

void stream_dispose(struct stream *s)
{
    if (s->pipe) {
        gst_element_set_state(s->pipe, GST_STATE_NULL);
        gst_object_unref(s->pipe);
        s->pipe = NULL;
    }

    if (s->src) {
        gst_object_unref(s->src);
        s->src = NULL;
    }

    if (s->sink) {
        gst_object_unref(s->sink);
        s->sink = NULL;
    }
}

static void spice_gstaudio_dispose(GObject *obj)
{
    spice_gstaudio *p;
    SPICE_DEBUG("%s", __FUNCTION__);
    p = SPICE_GSTAUDIO_GET_PRIVATE(obj);

    stream_dispose(&p->playback);
    stream_dispose(&p->record);

    if (p->pchannel != NULL) {
        g_signal_handlers_disconnect_by_func(p->pchannel,
                                             channel_event, obj);
        g_object_unref(p->pchannel);
        p->pchannel = NULL;
    }

    if (p->rchannel != NULL) {
        g_signal_handlers_disconnect_by_func(p->rchannel,
                                             channel_event, obj);
        g_object_unref(p->rchannel);
        p->rchannel = NULL;
    }
}

static void spice_gstaudio_init(SpiceGstAudio *pulse)
{
    spice_gstaudio *p;

    p = pulse->priv = SPICE_GSTAUDIO_GET_PRIVATE(pulse);
    memset(p, 0, sizeof(*p));
}

static void spice_gstaudio_class_init(SpiceGstAudioClass *klass)
{
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);

    gobject_class->finalize = spice_gstaudio_finalize;
    gobject_class->dispose = spice_gstaudio_dispose;

    g_type_class_add_private(klass, sizeof(spice_gstaudio));
}

static void record_new_buffer(GstAppSink *appsink, gpointer data)
{
    SpiceGstAudio *gstaudio = data;
    spice_gstaudio *p = gstaudio->priv;
    GstMessage *msg;

    g_return_if_fail(p != NULL);

    msg = gst_message_new_application(GST_OBJECT(p->record.pipe), NULL);
    gst_element_post_message(p->record.pipe, msg);
}

static void record_stop(SpiceRecordChannel *channel, gpointer data)
{
    SpiceGstAudio *gstaudio = data;
    spice_gstaudio *p = gstaudio->priv;

    SPICE_DEBUG("%s", __FUNCTION__);
    if (p->record.pipe)
        gst_element_set_state(p->record.pipe, GST_STATE_READY);
}

static gboolean record_bus_cb(GstBus *bus, GstMessage *msg, gpointer data)
{
    SpiceGstAudio *gstaudio = data;
    spice_gstaudio *p = gstaudio->priv;

    g_return_val_if_fail(p != NULL, FALSE);

    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_APPLICATION: {
        GstBuffer *b;

        b = gst_app_sink_pull_buffer(GST_APP_SINK(p->record.sink));
        if (!b) {
            if (!gst_app_sink_is_eos(GST_APP_SINK(p->record.sink)))
                g_warning("eos not reached, but can't pull new buffer");
            return TRUE;
        }

        spice_record_send_data(SPICE_RECORD_CHANNEL(p->rchannel),
                               /* FIXME: server side doesn't care about ts?
                                  what is the unit? ms apparently */
                               GST_BUFFER_DATA(b), GST_BUFFER_SIZE(b), 0);
        break;
    }
    default:
        break;
    }

    return TRUE;
}

static void record_start(SpiceRecordChannel *channel, gint format, gint channels,
                         gint frequency, gpointer data)
{
    SpiceGstAudio *gstaudio = data;
    spice_gstaudio *p = gstaudio->priv;

    g_return_if_fail(p != NULL);
    g_return_if_fail(format == SPICE_AUDIO_FMT_S16);

    if (p->record.pipe &&
        (p->record.rate != frequency ||
         p->record.channels != channels)) {
        record_stop(channel, data);
        gst_object_unref(p->record.pipe);
        p->record.pipe = NULL;
    }

    if (!p->record.pipe) {
        GError *error = NULL;
        GstBus *bus;
        gchar *audio_caps =
            g_strdup_printf("audio/x-raw-int,channels=%d,rate=%d,signed=(boolean)true,"
                            "width=16,depth=16,endianness=1234", channels, frequency);
        gchar *pipeline =
            g_strdup_printf("autoaudiosrc name=audiosrc ! queue ! audioconvert ! audioresample ! "
                            "appsink caps=\"%s\" name=appsink", audio_caps);

        p->record.pipe = gst_parse_launch(pipeline, &error);
        if (p->record.pipe == NULL) {
            g_warning("Failed to create pipeline: %s", error->message);
            goto lerr;
        }

        bus = gst_pipeline_get_bus(GST_PIPELINE(p->record.pipe));
        gst_bus_add_watch(bus, record_bus_cb, data);
        gst_object_unref(GST_OBJECT(bus));

        p->record.src = gst_bin_get_by_name(GST_BIN(p->record.pipe), "audiosrc");
        p->record.sink = gst_bin_get_by_name(GST_BIN(p->record.pipe), "appsink");
        p->record.rate = frequency;
        p->record.channels = channels;

        gst_app_sink_set_emit_signals(GST_APP_SINK(p->record.sink), TRUE);
        g_signal_connect(p->record.sink, "new-buffer",
                         G_CALLBACK(record_new_buffer), data);

lerr:
        g_clear_error(&error);
        g_free(audio_caps);
        g_free(pipeline);
    }

    if (p->record.pipe)
        gst_element_set_state(p->record.pipe, GST_STATE_PLAYING);
}

static void channel_event(SpiceChannel *channel, SpiceChannelEvent event,
                          gpointer data)
{
    SpiceGstAudio *gstaudio = data;
    spice_gstaudio *p = gstaudio->priv;

    switch (event) {
    case SPICE_CHANNEL_OPENED:
        break;
    case SPICE_CHANNEL_CLOSED:
        if (channel == p->pchannel) {
            p->pchannel = NULL;
            g_object_unref(channel);
        } else if (channel == p->rchannel) {
            record_stop(SPICE_RECORD_CHANNEL(channel), gstaudio);
            p->rchannel = NULL;
            g_object_unref(channel);
        } else /* if (p->pchannel || p->rchannel) */
            g_warn_if_reached();
        break;
    default:
        break;
    }
}

static void playback_stop(SpicePlaybackChannel *channel, gpointer data)
{
    SpiceGstAudio *gstaudio = data;
    spice_gstaudio *p = SPICE_GSTAUDIO_GET_PRIVATE(gstaudio);

    if (p->playback.pipe)
        gst_element_set_state(p->playback.pipe, GST_STATE_READY);
    if (p->mmtime_id != 0) {
        g_source_remove(p->mmtime_id);
        p->mmtime_id = 0;
    }
}

static gboolean update_mmtime_timeout_cb(gpointer data)
{
    SpiceGstAudio *gstaudio = data;
    spice_gstaudio *p = SPICE_GSTAUDIO_GET_PRIVATE(gstaudio);
    GstQuery *q;

    q = gst_query_new_latency();
    if (gst_element_query(p->playback.pipe, q)) {
        gboolean live;
        GstClockTime minlat, maxlat;
        gst_query_parse_latency(q, &live, &minlat, &maxlat);
        spice_playback_channel_set_delay(SPICE_PLAYBACK_CHANNEL(p->pchannel), GST_TIME_AS_MSECONDS(maxlat));
    }
    gst_query_unref (q);

    return TRUE;
}

static void playback_start(SpicePlaybackChannel *channel, gint format, gint channels,
                           gint frequency, gpointer data)
{
    SpiceGstAudio *gstaudio = data;
    spice_gstaudio *p = SPICE_GSTAUDIO_GET_PRIVATE(gstaudio);

    g_return_if_fail(p != NULL);
    g_return_if_fail(format == SPICE_AUDIO_FMT_S16);

    if (p->playback.pipe &&
        (p->playback.rate != frequency ||
         p->playback.channels != channels)) {
        playback_stop(channel, data);
        gst_object_unref(p->playback.pipe);
        p->playback.pipe = NULL;
    }

    if (!p->playback.pipe) {
        GError *error = NULL;
        gchar *audio_caps =
            g_strdup_printf("audio/x-raw-int,channels=%d,rate=%d,signed=(boolean)true,"
                            "width=16,depth=16,endianness=1234", channels, frequency);
        gchar *pipeline = g_strdup (g_getenv("SPICE_GST_AUDIOSINK"));
        if (pipeline == NULL)
            pipeline = g_strdup_printf("appsrc is-live=1 do-timestamp=0 caps=\"%s\" name=\"appsrc\" ! queue ! "
                                       "audioconvert ! audioresample ! autoaudiosink name=\"audiosink\"", audio_caps);
        SPICE_DEBUG("audio pipeline: %s", pipeline);
        p->playback.pipe = gst_parse_launch(pipeline, &error);
        if (p->playback.pipe == NULL) {
            g_warning("Failed to create pipeline: %s", error->message);
            goto lerr;
        }
        p->playback.src = gst_bin_get_by_name(GST_BIN(p->playback.pipe), "appsrc");
        p->playback.sink = gst_bin_get_by_name(GST_BIN(p->playback.pipe), "audiosink");
        p->playback.rate = frequency;
        p->playback.channels = channels;

lerr:
        g_clear_error(&error);
        g_free(audio_caps);
        g_free(pipeline);
    }

    if (p->playback.pipe)
        gst_element_set_state(p->playback.pipe, GST_STATE_PLAYING);

    if (p->mmtime_id == 0) {
        update_mmtime_timeout_cb(gstaudio);
        p->mmtime_id = g_timeout_add_seconds(1, update_mmtime_timeout_cb, gstaudio);
    }
}

static void playback_data(SpicePlaybackChannel *channel,
                          gpointer *audio, gint size,
                          gpointer data)
{
    SpiceGstAudio *gstaudio = data;
    spice_gstaudio *p = SPICE_GSTAUDIO_GET_PRIVATE(gstaudio);
    GstBuffer *buf;

    g_return_if_fail(p != NULL);

    audio = g_memdup(audio, size); /* TODO: try to avoid memory copy */
    buf = gst_app_buffer_new(audio, size, g_free, audio);
    gst_app_src_push_buffer(GST_APP_SRC(p->playback.src), buf);
}

static void channel_new(SpiceSession *s, SpiceChannel *channel, gpointer data)
{
    SpiceGstAudio *gstaudio = data;
    spice_gstaudio *p = gstaudio->priv;

    if (SPICE_IS_PLAYBACK_CHANNEL(channel)) {
        g_return_if_fail(p->pchannel == NULL);
        p->pchannel = g_object_ref(channel);
        g_signal_connect(channel, "playback-start",
                         G_CALLBACK(playback_start), gstaudio);
        g_signal_connect(channel, "playback-data",
                         G_CALLBACK(playback_data), gstaudio);
        g_signal_connect(channel, "playback-stop",
                         G_CALLBACK(playback_stop), gstaudio);
        g_signal_connect(channel, "channel-event",
                         G_CALLBACK(channel_event), gstaudio);
        spice_channel_connect(channel);
    }

    if (SPICE_IS_RECORD_CHANNEL(channel)) {
        g_return_if_fail(p->rchannel == NULL);
        p->rchannel = g_object_ref(channel);
        g_signal_connect(channel, "record-start",
                         G_CALLBACK(record_start), gstaudio);
        g_signal_connect(channel, "record-stop",
                         G_CALLBACK(record_stop), gstaudio);
        g_signal_connect(channel, "channel-event",
                         G_CALLBACK(channel_event), gstaudio);
        spice_channel_connect(channel);
    }
}

SpiceGstAudio *spice_gstaudio_new(SpiceSession *session, GMainContext *context,
                                  const char *name)
{
    SpiceGstAudio *gstaudio;
    spice_gstaudio *p;
    GList *list;

    gst_init(NULL, NULL);
    gstaudio = g_object_new(SPICE_TYPE_GSTAUDIO, NULL);
    p = SPICE_GSTAUDIO_GET_PRIVATE(gstaudio);
    p->session = g_object_ref(session);

    g_signal_connect(session, "channel-new",
                     G_CALLBACK(channel_new), gstaudio);
    list = spice_session_get_channels(session);
    for (list = g_list_first(list); list != NULL; list = g_list_next(list)) {
        channel_new(session, list->data, (gpointer)gstaudio);
    }
    g_list_free(list);

    return gstaudio;
}
