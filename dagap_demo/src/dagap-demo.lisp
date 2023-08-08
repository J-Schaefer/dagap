(in-package :dagap-demo)

(defparameter object-grasps
'((:spoon . :top)
(:breakfast-cereal . :front)
(:milk . :front)
(:cup . :top)
(:bowl . :top)))

(defparameter object-spawning-poses
'("sink_area_surface"
((:breakfast-cereal . ((0.2 -0.15 0.1) (0 0 0 1)))
(:cup . ((0.2 -0.35 0.1) (0 0 0 1)))
(:bowl . ((0.18 -0.55 0.1) (0 0 0 1)))
(:spoon . ((0.15 -0.4 -0.05) (0 0 0 1)))
(:milk . ((0.07 -0.35 0.1) (0 0 0 1)))))
"Relative poses on sink area")

(defun start-demo ()
  (roslisp:ros-info () "Starting DAGAP demo." ()))
