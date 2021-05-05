#include "sensorData.h"
namespace ml {

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION_SCANNET
#include "sensorData1/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION_SCANNET
#include "sensorData1//stb_image_write.h"
//#include <stb_image_write.h>

    void SensorData::RGBDFrame::compressDepth(const unsigned short *depth, unsigned int width, unsigned int height,
                                              COMPRESSION_TYPE_DEPTH type) {
        freeDepth();

        if (type == TYPE_RAW_USHORT) {
            if (m_depthSizeBytes != width * height) {
                freeDepth();
                m_depthSizeBytes = width * height * sizeof(unsigned short);
                m_depthCompressed = (unsigned char *) std::malloc(m_depthSizeBytes);
            }
            std::memcpy(m_depthCompressed, depth, m_depthSizeBytes);
        } else if (type == TYPE_ZLIB_USHORT) {
            freeDepth();

            int out_len = 0;
            int quality = 8;
            int n = 2;
            unsigned char *tmpBuff = (unsigned char *) std::malloc((width * n + 1) * height);
            std::memcpy(tmpBuff, depth, width * height * sizeof(unsigned short));
            m_depthCompressed = stbi_zlib_compress(tmpBuff, width * height * sizeof(unsigned short), &out_len,
                                                        quality);
            std::free(tmpBuff);
            m_depthSizeBytes = out_len;
        } else if (type == TYPE_OCCI_USHORT) {
            freeDepth();
#ifdef _USE_UPLINK_COMPRESSION
            //TODO fix the shift here
                        int out_len = 0;
                        int n = 2;
                        unsigned int tmpBuffSize = (width*n + 1) * height;
                        unsigned char* tmpBuff = (unsigned char *)std::malloc(tmpBuffSize);
                        out_len = uplinksimple::encode(depth, width*height, tmpBuff, tmpBuffSize);
                        m_depthSizeBytes = out_len;
                        m_depthCompressed = (unsigned char*)std::malloc(out_len);
                        std::memcpy(m_depthCompressed, tmpBuff, out_len);
                        std::free(tmpBuff);
#else
            throw MLIB_EXCEPTION("need UPLINK_COMPRESSION");
#endif
        } else {
            throw MLIB_EXCEPTION("unknown compression type");
        }
    }

    vec3uc* SensorData::RGBDFrame::decompressColorAlloc_stb(COMPRESSION_TYPE_COLOR type) const {	//can handle PNG, JPEG etc.
        if (type != TYPE_JPEG && type != TYPE_PNG) throw MLIB_EXCEPTION("invliad type");
        if (m_colorCompressed == NULL || m_colorSizeBytes == 0) throw MLIB_EXCEPTION("decompression error");
        int channels = 3;
        int width, height;
        unsigned char* raw = ScanNetstb::stbi_load_from_memory(m_colorCompressed, (int)m_colorSizeBytes, &width, &height, NULL, channels);

        return (vec3uc*)raw;
    }

    unsigned short* SensorData::RGBDFrame::decompressDepthAlloc_stb(COMPRESSION_TYPE_DEPTH type) const {
        if (type != TYPE_ZLIB_USHORT) throw MLIB_EXCEPTION("invliad type");
        unsigned short* res;
        int len;
        res = (unsigned short*)ScanNetstb::stbi_zlib_decode_malloc((const char*)m_depthCompressed, (int)m_depthSizeBytes, &len);
        return res;
    }
}