;; Auto-generated. Do not edit!


(when (boundp 'arm_module::ObjectDetection)
  (if (not (find-package "ARM_MODULE"))
    (make-package "ARM_MODULE"))
  (shadow 'ObjectDetection (find-package "ARM_MODULE")))
(unless (find-package "ARM_MODULE::OBJECTDETECTION")
  (make-package "ARM_MODULE::OBJECTDETECTION"))

(in-package "ROS")
;;//! \htmlinclude ObjectDetection.msg.html


(defclass arm_module::ObjectDetection
  :super ros::object
  :slots (_label _x _y _depth ))

(defmethod arm_module::ObjectDetection
  (:init
   (&key
    ((:label __label) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:depth __depth) 0.0)
    )
   (send-super :init)
   (setq _label (string __label))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _depth (float __depth))
   self)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:serialization-length
   ()
   (+
    ;; string _label
    4 (length _label)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _depth
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _depth
     (setq _depth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get arm_module::ObjectDetection :md5sum-) "6c811d8df461a90868e78656a89fbe92")
(setf (get arm_module::ObjectDetection :datatype-) "arm_module/ObjectDetection")
(setf (get arm_module::ObjectDetection :definition-)
      "# ObjectDetection.msg
string label
float32 x
float32 y
float32 depth


")



(provide :arm_module/ObjectDetection "6c811d8df461a90868e78656a89fbe92")


