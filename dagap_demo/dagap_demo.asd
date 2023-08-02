(asdf:defsystem dagap-demo
    :depends-on (roslisp std_msgs-msg actionlib actionlib_msgs-msg learning_actionlib-msg)
    :components
    ((:module "src"
              :components
              ((:file "package")
               (:file "fibonacci-server" :depends-on ("package"))
               (:file "fibonacci-client" :depends-on ("package"))))))
