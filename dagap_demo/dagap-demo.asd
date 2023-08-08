(asdf:defsystem dagap-demo
    :depends-on (roslisp roslisp-utilities std_msgs-msg actionlib actionlib_msgs-msg learning_actionlib-msg)
    :components
    ((:module "src"
              :components
              ((:file "package")
               (:file "dagap-demo" :depends-on ("package"))))))
