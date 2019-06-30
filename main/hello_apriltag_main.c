#include <stdio.h>
#include "esp_log.h"

//esp32-camera
#include "esp_camera.h"

// apriltag
#include "apriltag.h"
#include "tag25h9.h"
#include "common/image_u8.h"
#include "common/zarray.h"

static const char* TAG = "camera";

//M5STACK_CAM PIN Map
#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   15 //software reset will be performed
#define CAM_PIN_XCLK    27
#define CAM_PIN_SIOD    22
#define CAM_PIN_SIOC    23

#define CAM_PIN_D7      19
#define CAM_PIN_D6      36
#define CAM_PIN_D5      18
#define CAM_PIN_D4      39
#define CAM_PIN_D3      5
#define CAM_PIN_D2      34
#define CAM_PIN_D1      35
#define CAM_PIN_D0      32

#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    26
#define CAM_PIN_PCLK    21

#define CAM_XCLK_FREQ   20000000

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QQVGA,//Do not use sizes above QVGA when not JPEG

    // .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1 //if more than one, i2s runs in continuous mode.
};


void app_main()
{
    apriltag_family_t *tf = tag25h9_create();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_sigma = 0.0;
    td->quad_decimate = 1.0;
    td->refine_edges = 0;
    td->decode_sharpening = 0;
    td->nthreads = 1;
    td->debug = 0;

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }

    while(1){
        //acquire a frame
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera Capture Failed");
            return;
        }

        image_u8_t im = {
            .width = fb->width,
            .height = fb->height,
            .stride = fb->width,
            .buf = fb->buf
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            printf("%d, ",det->id);
        }
        printf("\n");

        apriltag_detections_destroy(detections);

        double t =  timeprofile_total_utime(td->tp) / 1.0E3;
        printf("%12.3f \n", t);

        //return the frame buffer back to the driver for reuse
        esp_camera_fb_return(fb);

    }

    // don't deallocate contents of inputs; those are the argv
    apriltag_detector_destroy(td);

    tag25h9_destroy(tf);

}
