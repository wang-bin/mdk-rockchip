/*
 * Copyright (c) 2025 WangBin <wbsecg1 at gmail.com>
 * This file is part of MDK
 * MDK SDK: https://github.com/wang-bin/mdk-sdk
 * Free for opensource softwares or non-commercial use.
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 */
#include "mdk/VideoDecoder.h"
#include "mdk/MediaInfo.h"
#include "mdk/Packet.h"
#include "mdk/VideoFrame.h"
#include <cmath>
#include <iostream>
#include "base/log.h"
#include "base/scope_atexit.h"
#include "video/DRMPrime.h"

extern "C" {
#include <drm_fourcc.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_log.h>
#include <rockchip/rk_mpi.h>
}

/*
export mpp_dec_debug=1 mpi_debug=1 buf_slot_debug=1 mpp_buffer_debug=1 mpp_task_debug=1 mpp_meta_debug=1 h264d_debug=1 hal_h264d_debug=1 hal_bufs_debug=1 drm_debug=1 ion_debug=1
export mpp_log_level=1
export  mpp_debug=23 # MPP_DBG_INFO | ...
*/

typedef void (*MppLogCb)(void *ctx, int level, const char *tag, const char *fmt, const char *func, va_list args);
extern "C" int mpp_set_log_callback(void *ctx, MppLogCb cb);
_Pragma("weak mpp_set_log_callback")

using namespace std;

#define RK_ENSURE(expr, ...) RK_RUN(expr, return __VA_ARGS__)
#define RK_WARN(expr, ...) RK_RUN(expr)
#define RK_RUN(EXPR, ...) do { \
        const auto rk_ret__ = (EXPR); \
        if (rk_ret__ != RK_OK) { \
            std::clog << fmt::to_string("RK ERROR@%d %s " #EXPR " : %d", __LINE__, __func__, rk_ret__) << std::endl << std::flush; \
            __VA_ARGS__; \
        } \
    } while (false)


MDK_NS_BEGIN

static MppCodingType codec_from(string_view name)
{
    if (name == "h264" || name == "avc") {
        return MPP_VIDEO_CodingAVC;
    } else if (name == "h265" || name == "hevc") {
        return MPP_VIDEO_CodingHEVC;
    } else if (name == "vp8") {
        return MPP_VIDEO_CodingVP8;
    } else if (name == "vp9") {
        return MPP_VIDEO_CodingVP9;
    } else if (name == "av1") {
        return MPP_VIDEO_CodingAV1;
    } else if (name.ends_with("jpeg")) {
        return MPP_VIDEO_CodingMJPEG;
    } else if (name == "h263") {
        return MPP_VIDEO_CodingH263;
    } else if (name == "mpeg4") {
        return MPP_VIDEO_CodingMPEG4;
    } else if (name.starts_with("mpeg")) { // mpeg1/2
        return MPP_VIDEO_CodingMPEG2;
    }
    return MPP_VIDEO_CodingUnused;
}

// bpc = 8, 10 no 12
static PixelFormat from(MppFrameFormat mf)
{
    using enum PixelFormat;
    switch (mf & MPP_FRAME_FMT_MASK) {
    case MPP_FMT_YUV420SP: return NV12;
    case MPP_FMT_YUV420SP_10BIT: return N010;
    case MPP_FMT_YUV422SP: return NV16;
    case MPP_FMT_YUV422SP_10BIT: return N210;
    case MPP_FMT_YUV444SP: return NV24;
    case MPP_FMT_YUV444SP_10BIT: return P410;
    default: return Unknown;
    }
}

static uint32_t to_drm(MppFrameFormat mpp_fmt)
{
    if (MPP_FRAME_FMT_IS_FBC(mpp_fmt)) {
        switch (mpp_fmt & MPP_FRAME_FMT_MASK) {
        case MPP_FMT_YUV420SP:          return DRM_FORMAT_YUV420_8BIT; // 1 plane yuv, undefined layout, non-linear modifier is required, e.g. afbc
        case MPP_FMT_YUV420SP_10BIT:    return DRM_FORMAT_YUV420_10BIT; // 1 plane yuv, undefined layout, non-linear modifier is required, e.g. afbc
        case MPP_FMT_YUV422SP:          return DRM_FORMAT_YUYV;
        case MPP_FMT_YUV422SP_10BIT:    return DRM_FORMAT_Y210;
        case MPP_FMT_YUV444SP:          return fourcc_code('V', 'U', '2', '4'); // DRM_FORMAT_VUY888. nv24 afbc
        default:                        return DRM_FORMAT_INVALID;
        }
    }
    switch (mpp_fmt & MPP_FRAME_FMT_MASK) {
    case MPP_FMT_YUV420SP:          return DRM_FORMAT_NV12;
    case MPP_FMT_YUV420SP_10BIT:    return fourcc_code('N', 'V', '1', '5'); // p010 w/o gaps
    case MPP_FMT_YUV422SP:          return DRM_FORMAT_NV16;
    case MPP_FMT_YUV422SP_10BIT:    return fourcc_code('N', 'V', '2', '0');
    case MPP_FMT_YUV444SP:          return DRM_FORMAT_NV24;
    default:                        return DRM_FORMAT_INVALID;
    }
}

static bool to_drm(drm::Frame& df, MppFrame mf)
{
    if (!mf) {
        clog << "ERROR: rockchip mpp get 0 frame" << endl;
        return false;
    }
    auto buf = mpp_frame_get_buffer(mf);
    if (!buf) {
        clog << "ERROR: MppFrame no buffer" << endl;
        return false;
    }
    const auto mfmt = mpp_frame_get_fmt(mf);
    VideoFormat fmt = from(mfmt);
    if (!fmt) {
        clog << fmt::to_string("ERROR: rockchip mpp unsupported format: %d", mfmt) << endl;
        return false;
    }
    const bool fbc = mfmt & MPP_FRAME_FBC_MASK;
    df.width = mpp_frame_get_width(mf);
    df.height = mpp_frame_get_height(mf);
    df.format = fmt;
    df.owner = false; // dup() and owner?
    df.objects.resize(1);
    df.layers.resize(1);
    df.objects[0].fd = mpp_buffer_get_fd(buf);
    df.objects[0].size = mpp_buffer_get_size(buf);
    if (fbc) { // rfbc?
        df.objects[0].format_modifier = DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_SPARSE | AFBC_FORMAT_MOD_BLOCK_SIZE_16x16);
    }
    auto& layer = df.layers[0];
    layer.format = to_drm(mfmt);
    layer.planes.resize(fbc ? 1 : 2);
    layer.planes[0].object_index = 0;
    layer.planes[0].offset = 0;
    layer.planes[0].pitch = mpp_frame_get_hor_stride(mf);
    if (layer.planes.size() > 1) {
        layer.planes[1].object_index = 0;
        layer.planes[1].offset = mpp_frame_get_ver_stride(mf) * layer.planes[0].pitch;
        layer.planes[1].pitch = layer.planes[0].pitch * fmt.subsampleWidth(4) / 2; // * 1 for 420 422, 2 for 444
    }
    return true;
}

static void set_properties(VideoFrame& f, MppFrame mf)
{
    if (mpp_frame_get_eos(mf)) {
        f.setTimestamp(TimestampEOS);
        clog << "rockchip mpp EOS frame" << endl;
        /* EOS frame may contain valid data */
        if (!mpp_frame_get_buffer(mf)) {
            return;
        }
    } else {
        f.setTimestamp(double(mpp_frame_get_pts(mf))/FrameTimeScaleForInt);
    }
    // optional because will set by FrameReader
    ColorSpace cs;
    cs.primaries = ColorSpace::Primary(mpp_frame_get_color_primaries(mf));
    cs.transfer = ColorSpace::Transfer(mpp_frame_get_color_trc(mf));
    cs.matrix = ColorSpace::Matrix(mpp_frame_get_colorspace(mf));
    cs.range = ColorSpace::Range(mpp_frame_get_color_range(mf));
    f.setColorSpace(cs, true);

    const auto mode = mpp_frame_get_mode(mf) & MPP_FRAME_FLAG_FIELD_ORDER_MASK;
    if (mode & MPP_FRAME_FLAG_DEINTERLACED) {
        f.setFieldStruct(mode & MPP_FRAME_FLAG_TOP_FIRST ? VideoFrame::FieldStruct::TopBottom : VideoFrame::FieldStruct::TopBottom);
    }
    // TODO: hdr metadata
}

class MppDecoder final : public VideoDecoder
{
public:
    MppDecoder() {
        set(Options::ToAnnexB
            | Options::InsertCSD
            | Options::BaseLayer | Options::Default);
        pool_ = NativeVideoBufferPool::create("DRM"); // optional
            mpp_set_log_level(MPP_LOG_DEBUG);
        /*if (mpp_set_log_callback) // env var mpp_debug and mpp_log_level are enough
        mpp_set_log_callback(this, [](void *ctx, int level, const char *tag, const char *fmt, const char *func, va_list args) {
            va_list tmp;
            va_copy(tmp, args);
            std::string vamsg(std::vsnprintf(nullptr, 0, fmt, tmp), 0);
            std::vsnprintf(&vamsg[0], vamsg.size() + 1, fmt, args);// +1 for vsnprintf terminating null, not not for std::string
            if (vamsg[vamsg.size() - 1] == '\n')
                vamsg.resize(vamsg.size() - 1);
            va_end(tmp); // required
            clog << fmt::to_string("rockchip mpp log [%s] %s: %s", tag, func, vamsg) << endl;
        });*/
    }

    const char* name() const override {return "rockchip";}
    bool open() override;
    bool close() override {
        if (mpi_ && ctx_) {
            RK_WARN(mpi_->reset(ctx_));
            RK_WARN(mpp_buffer_group_put(frame_group_)); // TODO: for external
            frame_group_ = nullptr;
            RK_WARN(mpp_destroy(ctx_));
            ctx_ = nullptr;
            mpi_ = nullptr;
        }
        onClose();
        return true;
    }
    bool flush() override;
    int decode(const Packet& pkt) override;
private:
    int processOutput();
    int processOutput(MppFrame mf);

    MppCtx ctx_ = {};
    MppApi *mpi_ = nullptr;
    MppBufferGroup frame_group_ = {};

    NativeVideoBufferPoolRef pool_;
};

bool MppDecoder::open()
{
    RK_ENSURE(mpp_create(&ctx_, &mpi_), false);
    clog << fmt::to_string("rockchip mpp api version: %u", mpi_->version) << endl;

    const auto& par = parameters();
    const auto codec = codec_from(par.codec);
    clog << fmt::to_string("rockchip mpp codec: %u", codec) << endl;
    RK_ENSURE(mpp_check_support_format(MPP_CTX_DEC, codec), false);
    RK_ENSURE(mpp_init(ctx_, MPP_CTX_DEC, codec), false);
    RK_S64 sync_timeout = std::stoll(get_or("sync", "0")); // 0: async, >0: sync/block mode with timeout(ms). -1: infinite timeout
    RK_ENSURE(mpi_->control(ctx_, MPP_SET_OUTPUT_TIMEOUT, &sync_timeout), false);
    auto prop = get_or("buffer", "drm");
    // TODO: +flags MPP_BUFFER_FLAGS
    MppBufferType type = MPP_BUFFER_TYPE_DRM;
    if (prop == "drm") {
        type = MPP_BUFFER_TYPE_DRM;
    } else if (prop == "ion") {
        type = MPP_BUFFER_TYPE_ION;
    } else if (prop == "normal") {
        type = MPP_BUFFER_TYPE_NORMAL;
    } else if (prop == "ext_dma") {
        type = MPP_BUFFER_TYPE_EXT_DMA; // provided by user
    } else if (prop == "dma_heap") {
        type = MPP_BUFFER_TYPE_DMA_HEAP;
    }

    int flags = 0;
    auto to_flag = [&flags](const auto& flag) {
        if (flag == "cachable") {
            flags |= MPP_BUFFER_FLAGS_CACHABLE; // DMA_BUF_IOCTL_SYNC in map/unmap
        } else if (flag == "secure") {
            flags |= MPP_BUFFER_FLAGS_SECURE;
        } else if (flag == "cma") {
            flags |= MPP_BUFFER_FLAGS_CONTIG;
        } else if (flag == "dma32") {
            flags |= MPP_BUFFER_FLAGS_DMA32;
        } else if (flag == "wc") {
            flags |= MPP_BUFFER_FLAGS_WC;
        } else if (flag == "kmap") {
            flags |= MPP_BUFFER_FLAGS_ALLOC_KMAP;
        }
        cout << fmt::to_string("mpp buf flag: %s = %#X", string(flag), flags) << endl;
    };
    prop = get("flags");
    if (!prop.empty()) {
        size_t f0 = 0;
        size_t f = prop.find('+');
        while (f != std::string::npos) {
            if (f != f0) {
                auto flag = string_view(&prop[f0], f - f0);
                to_flag(flag);
            }
            f0 = f + 1; // skip '+'
            f = prop.find('+', f0);
        }
        to_flag(string_view(&prop[f0]));
    }
    // TODO: buffer mode internal, external
    RK_ENSURE(mpp_buffer_group_get_internal(&frame_group_, type | flags), false);
    RK_ENSURE(mpi_->control(ctx_, MPP_DEC_SET_EXT_BUF_GROUP, frame_group_), false);
    size_t maxBufSize = 0;
    int maxBufCount = std::stoi(get_or("bufs", "16"));
    RK_ENSURE(mpp_buffer_group_limit_config(frame_group_, maxBufSize, maxBufCount), false);

    int deinterlace = std::stoi(get_or("deinterlace", "0"));
    RK_WARN(mpi_->control(ctx_, MPP_DEC_SET_ENABLE_DEINTERLACE, &deinterlace));
    int afbc = std::stoi(get_or("afbc", "0"));
    if (afbc) {
        // codec is h264, hevc, vp9, av1?
        int afbc_fmt = MPP_FRAME_FBC_AFBC_V2; // for video output
        RK_ENSURE(mpi_->control(ctx_, MPP_DEC_SET_OUTPUT_FORMAT, &afbc_fmt), false);
    }

    // TODO: MPP_DEC_SET_IMMEDIATE_OUT? MPP_DEC_SET_VC1_EXTRA_DATA
    // MPP_DEC_SET_DISABLE_THREAD: decode in caller thread
    // MPP_DEC_GET_VPUMEM_USED_COUNT, MPP_DEC_SET_ENABLE_MVC

    preprocess(par.extra);

    onOpen();
    return true;
}

bool MppDecoder::flush()
{
    if (!mpi_ || !ctx_) {
        return false;
    }
    RK_WARN(mpi_->reset(ctx_));
    onFlush();
    return true;
}

int MppDecoder::decode(const Packet& pkt)
{
    auto filtered = pkt.buffer;
    if (pkt.buffer && pkt.buffer->constData())
        filtered = preprocess(pkt);
    MppPacket mp;
    if (pkt.isEnd() || !filtered) {
        clog << "mpp packet EOS" << endl;
        RK_ENSURE(mpp_packet_init(&mp, nullptr, 0), {});
        mpp_packet_set_eos(mp);
    } else {
        // print filtered first 8 bytes
        RK_ENSURE(mpp_packet_init(&mp, filtered->data(), filtered->size()), {});
        mpp_packet_set_pts(mp, (RK_S64)(pkt.pts * FrameTimeScaleForInt));
        //mpp_packet_set_dts(mp, (RK_S64)(pkt.dts * FrameTimeScaleForInt));
    }

    int ret = 0;
    {
    [[maybe_unused]] const auto _ = scope_atexit([&mp]() {
        RK_WARN(mpp_packet_deinit(&mp));
    });
#if 1
    MPP_RET err;
    err = mpi_->decode_put_packet(ctx_, mp);
    if (err != MPP_OK) {
        if (err != MPP_ERR_BUFFER_FULL) {
            clog << fmt::to_string("rockchip mpp put packet failed: %d", err) << endl;
            return -1; // error
        }
    }
#else
    MppFrame mf = nullptr;
    RK_ENSURE(mpi_->decode(ctx_, mp, &mf), -1);
    processOutput(mf);
#endif
    ret = mpp_packet_get_length(mp); // bytes undecoded
    }

#if 1
    while (true) {
        auto out = processOutput();
        if (out < 0) {
            return -1; // error
        } else if (out == 0) {
            break;
        }
    }
#endif
    return !pkt.isEnd() ? ret : INT_MAX;
}

int MppDecoder::processOutput()
{
    MppFrame mf = nullptr;
    MPP_RET err;
    err = mpi_->decode_get_frame(ctx_, &mf);
    if (err != MPP_OK) {
        if (err != MPP_ERR_TIMEOUT && err != MPP_NOK) {
            clog << fmt::to_string("rockchip mpp decode get frame failed: %d. mf: %p", err, mf) << endl;
            return -1; // err == -1(MPP_NOK) if no more frame for sync mode(master branch mpp), should be -8(MPP_ERR_TIMEOUT)
        }
    }
    if (!mf) { // async: err == 0 if no output
        //clog << fmt::to_string("rockchip mpp decode get 0 frame. timeout = %d err: %d", err == MPP_ERR_TIMEOUT, err) << endl;
        return 0; // no output
    }
    return processOutput(mf);
}

int MppDecoder::processOutput(MppFrame mf)
{
    [[maybe_unused]] const auto _ = scope_atexit([&mf]() {
        if (mf)
            RK_WARN(mpp_frame_deinit(&mf));
    });
    if (mpp_frame_get_info_change(mf)) {
        const auto w = mpp_frame_get_width(mf);
        const auto h = mpp_frame_get_height(mf);
        clog << fmt::to_string("rockchip mpp frame info change to: %dx%d format: %d", w, h, mpp_frame_get_fmt(mf)) << endl;
        RK_WARN(mpi_->control(ctx_, MPP_DEC_SET_INFO_CHANGE_READY, nullptr));
        return 0;
    }
    // if info change, setup decoder info. for mjpeg, get frame now, otherwise eagain
    if (mpp_frame_get_discard(mf)) {
        clog << "rockchip mpp discard frame" << endl;
        return 0;
    }
    if (auto err = mpp_frame_get_errinfo(mf)) { // TODO: drop frame but ignore error
        clog << fmt::to_string("rockchip mpp frame error: %d", err) << endl;
        return -3; // error
    }

    drm::Frame df;
    if (!to_drm(df, mf)) {
        return -3; // EOS?
    }
    if (auto vf = VideoFrame::from(&pool_, df)) {
        vf.onDestroy([mf]() mutable {
            RK_WARN(mpp_frame_deinit(&mf));
        });
        set_properties(vf, mf);
        frameDecoded(std::move(vf));
        mf = nullptr; // avoid deinit
    } else {
        clog << "rockchip mpp decode get frame failed: no video frame" << endl;
        return -2; // error
    }
    return 1; // output one frame
}

static void register_video_decoders_rockchip() {
    VideoDecoder::registerOnce("rockchip", []{return new MppDecoder();});
}
MDK_NS_END

// project name must be rockchip or mdk-rockchip
MDK_PLUGIN(rockchip) {
    using namespace MDK_NS;
    register_video_decoders_rockchip();
    return MDK_ABI_VERSION;
}