# Observe default model

- Model: OpenCV Zoo YOLOX-S, COCO 80 classes, float32 ONNX, 640x640 input.
- Source: https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox
- Download: https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/object_detection_yolox/object_detection_yolox_2022nov.onnx
- SHA256: c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063
- Size: 35,858,002 bytes.
- License: Apache-2.0, see LICENSE-YOLOX (including upstream notices).
- Preprocessing: RGB float32 0..255, aspect-preserving resize, top-left placement,
  right/bottom padding 114. No division by 255. Output [1,8400,85], grid/stride
  decode, objectness x class score, class-aware NMS.
- OpenCV Zoo reports COCO val AP50:95=0.405; this is not a claim about aerial
  payload footage. Small aerial targets and infrared require application-specific
  data, validation and potentially fine-tuned weights. “Other” means the remaining
  known COCO classes, not an open-set unknown-object detector.

The model binary is kept beside the node XML so source and installed bringup
configurations work without downloading anything at runtime.
