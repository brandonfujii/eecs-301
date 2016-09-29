
(cl:in-package :asdf)

(defsystem "fw_wrapper-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "allcmd" :depends-on ("_package_allcmd"))
    (:file "_package_allcmd" :depends-on ("_package"))
    (:file "getcmd" :depends-on ("_package_getcmd"))
    (:file "_package_getcmd" :depends-on ("_package"))
  ))