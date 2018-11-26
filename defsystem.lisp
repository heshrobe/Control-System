;;; -*- Syntax: Ansi-common-lisp; Package: cl-USER; Base: 10; Mode: LISP -*- 

(in-package :cl-user)

(eval-when (:execute :load-toplevel)
  (let* ((loading-file *load-truename*)
	 (host (pathname-host loading-file))
	 (device (pathname-device loading-file))
	 (code-dir (pathname-directory loading-file))
	 (home-dir code-dir)
	 (wild-dir (append home-dir (list :wild-inferiors))))
    (let ((home-directory (make-pathname :directory home-dir
					 :host host
					 :device device))
	  (code-directory (make-pathname :directory code-dir
					 :host host
					 :device device))
	  (wild-directory (make-pathname :directory wild-dir
					 :host host 
					 :device device
					 :type :wild
					 :name :wild
					 :version :unspecific)))
      (setf (logical-pathname-translations "controls")
	`(("code;*.*" ,code-directory)
	  ("home;*.*" ,home-directory)
	  ("**;*.*"   ,wild-directory)
	  ))
      (with-open-file (F #P"controls:home;my-logical-pathnames.lisp" 
		       :direction :output
		       :if-does-not-exist :create

		       :if-exists :supersede)
	(format f "~%;;; controls")
	(format f "~2%~s" "controls")
	(loop for (a b) in (logical-pathname-translations "controls")
	  do (format f "~%'(~s ~s)" (namestring a) (namestring b)))
	(terpri f)
	)      
      (pushnew (namestring (truename #P"controls:home;my-logical-pathnames.lisp"))
	       (logical-pathname-translations-database-pathnames)
	       :test #'string-equal)
      )
    ))

(ql:quickload "cl-randist")

(defsystem controls
    (:default-pathname "controls:code;"
	:default-module-class separate-destination-module)
  (:serial
   ("trace-format" (:module-class separate-destination-module))
   ("plot" (:module-class separate-destination-module))
   ("kalman" (:module-class separate-destination-module))
   ("simulator" (:module-class separate-destination-module))
   ("wrapping" (:module-class separate-destination-joshua-module))
   ("paper-simulation" (:module-class separate-destination-module))
   ))









