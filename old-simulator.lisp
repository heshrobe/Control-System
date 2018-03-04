;;; -*- Mode: Commonlisp; package: Awdrat; Readtable: Joshua -*-

(in-package :awdrat)

;;; This is copied from a file in the clim-fixes system that qcri doesn't 
;;; yet have.  I use with-output-centered below, so I'm including it here
(eval-when (:compile-toplevel :load-toplevel)
  (export 'clim::with-output-centered 'clim))

;;; for some reason, emacs formats this correctly but not centering-output
(defmacro clim-internals::with-output-centered ((stream &key (move-cursor t)) &body body)
  (clim-internals::default-output-stream stream centering-output)
  `(flet ((centering-output-body (,stream) ,@body))
     (declare (dynamic-extent #'centering-output-body))
     (clim-internals::invoke-centering-output
      ,stream #'centering-output-body 
      :move-cursor ,move-cursor)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Useful Macros to make writing this clearer
;;; Deals with integration, differentiation and Updating state-variables
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(eval-when (:compile-toplevel :load-toplevel)
   (defun new-version-of (name)
    (intern (string-upcase (format nil "new-~a" name)))))

(defmacro integral-slice (value time-slice)
  (let ((new-value (new-version-of value)))
    `(* (/ (+ ,value ,new-value) 2) ,time-slice)))

(defmacro derivative-slice (value time-slice)
  (let ((new-value (new-version-of value)))
    `(/ (- ,new-value ,value) ,time-slice)))

(defmacro update-values (&rest value-names)
  (loop for v in value-names
      for new-v = (new-version-of v)
      collect v into sets
      collect new-v into sets
      finally (return `(setq ,@sets))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Definition of a generic parameterizable PID controller
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass pid-controller ()
  ((feedback :initform 0 :accessor feedback)
   (signal-error :initform 0 :accessor signal-error)
   (proportional :initform 0 :accessor proportional)
   (derivative :initform 0 :accessor derivative)
   (integral :initform 0 :accessor integral)
   (kp :initarg :kp :accessor kp :initform 1)
   (ki :initarg :ki :accessor ki :initform 0.01)
   (kd :initarg :kd :accessor kd :initform 0.01)
   (setpoint :initarg :setpoint :accessor setpoint :initform 5)
   (lower-limit :initarg :lower-limit :accessor lower-limit :initform nil)
   (upper-limit :initarg :upper-limit :accessor upper-limit :initform nil)
   ))


(defmethod reset ((controller pid-controller))
  (setf (integral controller) 0))

(defmethod respond-to-sensor-value ((controller pid-controller) observation dt)
  (with-slots (signal-error proportional derivative integral total-integral kp ki kd setpoint upper-limit lower-limit) controller
    (let* ((new-signal-error (- observation setpoint))
	   (new-proportional new-signal-error)
	   (new-derivative (derivative-slice  signal-error dt))
	   (new-integral (integral-slice signal-error dt)))
      (incf integral new-integral)
      (update-values signal-error signal-error proportional derivative)
      (flet ((clip (value)
	       (when lower-limit
		 (setq value (max lower-limit value)))
	       (when upper-limit
		 (setq value (min upper-limit value)))
	       value))
	(values (clip (- (+ (* kp proportional) (* ki integral) (* kd derivative))))
		signal-error)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; The driver function and its supporting macro
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro observe (&rest variable-names)
  (flet ((make-sequence-name (variable-name)
	   (intern (string-upcase (format nil "~a-sequence" variable-name)))))
    `(progn ,@(loop for variable-name in variable-names
		  collect `(vector-push-extend ,variable-name ,(make-sequence-name variable-name))))))

;;; The core routine for running a simulation and collecting data of interest
;;; ToDo: There's a time step skew here.  Plant outputs should be one step later.
(defun do-a-simulation (plant controller time dt signal-distortion signal-labels plant-output-name)
  ;; reset the running sum of the integral in the PID controller
  (reset controller)
  (let* ((number-of-steps (ceiling time dt))
	 (error-sequence (make-array number-of-steps :fill-pointer 0 :adjustable t))
	 (sensor-sequence (make-array number-of-steps :fill-pointer 0 :adjustable t))
	 (command-sequence (make-array number-of-steps :fill-pointer 0 :adjustable t))
	 (plant-signal-alist  (loop for name in signal-labels collect (list name (make-array number-of-steps :fill-pointer 0 :adjustable t))))
	 (full-signal-alist (append `((error ,error-sequence) (sensor ,sensor-sequence) (command ,command-sequence))
				    plant-signal-alist))
	 (initial-output (initial-output plant)))
    ;; The simulation will start by initializing the plant on time step 0 and geting its outputs
    ;; and then calling the controller on time step 0.  
    ;; Then on each succeeding time step we pass the controller command
    ;; to the plant and get its output.
    (loop for time from 0 by dt below time
	for command = nil then next-command
	for plant-output = (let ((plant-observation-alist (if (= time 0) (initialize plant initial-output) (respond-to-command plant command dt))))
			     ;; now observe everything the plant told us
			     (loop for (name vector) in plant-signal-alist
				 for observation = (second (assoc name plant-observation-alist))
				 do (vector-push-extend observation vector))
			     ;; and then return the plant's output from the alist
			     (second (assoc plant-output-name plant-observation-alist)))
	for sensor = (if signal-distortion (funcall signal-distortion plant-output time) plant-output)
	for next-command = (multiple-value-bind (command error) (respond-to-sensor-value controller sensor dt)
			     ;; and observe the sensor-value, error, and controller's command
			     (observe error sensor command)
			     command)
	until (and (awdrat-enabled? clim:*application-frame*)
		   (error-detected (awdrat-information clim:*application-frame*)))
	      )
    full-signal-alist))

;;; Top level driver for standalone use to interface to plotting code
(defun oo-do-it (plant controller 
		 &key
		 (time 10) (dt 0.01)
		 ;; The function for lying about the plant output
		 (signal-distortion #'(lambda (value time) (declare (ignore time)) value))
		 ;; plotting parameters
		 (plot-colors (LIST CLIM:+RED+ CLIM:+BLUE+ CLIM:+GREEN+ +ORANGE+ +VIOLET+ +light-blue+))
		 (signal-labels '(height sensor error command pump-rate))
		 plant-output-name
		 )
  (let ((results (do-a-simulation plant controller time dt signal-distortion signal-labels plant-output-name)))
    (plot (mapcar #'second results)
	  :x-grid-increment 1 :y-grid-increment 1 
	  :x-scale 30
	  :y-scale 30 
	  :x-value-increment dt
	  :colors plot-colors
	  :parameter-names (list* 'error 'sensor 'command signal-labels)
	  )))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Some Useful signal distorters
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; I used to use a "factory" pattern when I stored previously made distorters in a lookup
;;; list.  I simplified this so you make a new copy on every new specification.

(defun make-a-fixed-offset-distorter (offset)
  (flet ((distorter (real-value time) 
		 (declare (ignore time))
		 (- real-value offset)))
	  #'distorter))

(defun make-an-increasing-offset-distorter (start-time stop-time total-amount)
  (let ((start-time start-time)
	(slope (/ (float total-amount) (- stop-time start-time)))
	(stop-time stop-time))
    (flet ((distorter (real-value time)
	     (cond
	      ((< time start-time) real-value)
	      ((<= start-time time stop-time)
	       (- real-value (* slope (- time start-time))))
	      ((> time stop-time)
	       (- real-value (* slope (- stop-time start-time)))))))
      #'distorter)))

(defun make-a-staged-percent-distorter (initial-percent delta-percent sample-rate)
  (let ((percent initial-percent))
    (flet ((distorter (real-value time)
	     (when (= time 0)
	       (setq percent initial-percent))
	     (when (zerop (mod time sample-rate))
	       (incf percent delta-percent))
	     (- real-value (* percent real-value))))
      #'distorter)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; A watertank plant model
;;;
;;; The plant has a pump that feeds in at rate b * V (the control signal which is a voltage, b a scaling parameter)
;;; The plant has an outflow at rate a * sqrt(H) where H is the height of the water in the tank (proportional to pressure)
;;; and a is a parameter (related to the area of the outflow pipe, I guess).
;;; The tank has cross sectional area A and Volume Vol, so Vol = A * H
;;; The differential equation for the tank is then:
;;; d(H, t) = b * V - a * sqrt(h)
;;;           ------------------
;;;                  A
;;;
;;;  The default model below has:
;;;      Area = 10 sq ft
;;;      Height = 10 ft.
;;;      Volume = 100 cu ft
;;;      Drain rate = 0.1 cu ft/sec/sqrt(ft) * sqrt(height)
;;;      Pump rate = 2 cu ft/sec/volt * control-signal
;;;      With a control of 1 volt, the tank would make up 1 ft of height in 5 sec, ignoring the drain
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass water-tank-plant-model ()
  (;; First are the parameters describing the system
   ;; Area is the cross-sectional area of the tank in square feet
   (Area :accessor area :initarg :area :initform 10)
   ;; the proportionality constant for the pump in cu-ft/sec/volt (i.e. a 1 volt input causes the pump to move 1 cu-ft/sec
   (pump-rate-scale :accessor pump-rate-scale :initarg :pump-rate-scale :initform 2)
   ;; the proportionality constant for the drain-hole in cu-ft/sec/exp(ft,.5)
   (drain-rate :accessor drain-rate :initarg :drain-rate :initform 2)
   ;; Now the state-variables
   ;; the height of the water in the tank
   (height :accessor height :initform 10 :initarg :height)
   ;; the volume redundant with height, but who cares
   (volume :accessor volume)
   ;; the old pump rate
   (pump-rate :accessor pump-rate :initform 0)
   ))

(defmethod initialize-instance :after ((tank water-tank-plant-model) &rest args)
  (declare (ignore args))
  (setf (volume tank) (* (height tank) (area tank))))

(defmethod (setf height) :after (new-value (tank water-tank-plant-model))
  (setf (volume tank) (* new-value (area tank))))

(defmethod initialize ((tank water-tank-plant-model) initial-value)
  (with-slots (height pump-rate pump-rate-scale drain-rate volume area) tank
    ;; using setf so that the setf method calculates the volume
    (setf (height tank) initial-value)
    (let ((current-drain-rate (* (sqrt height) drain-rate)))
      `((height ,height) (volume ,volume) (pump-rate 0) (drain-rate ,current-drain-rate)))))

(defmethod initial-output ((tank water-tank-plant-model))
  (height tank))

(defmethod respond-to-command ((tank water-tank-plant-model) control dt)
  (with-slots (height pump-rate pump-rate-scale drain-rate volume area) tank
    (let* ((new-pump-rate (* pump-rate-scale control))
	   (pump-volume-increment (integral-slice pump-rate dt))
	   ;; notice this is using only the old value since we're calculating
	   ;; the new value using this
	   (current-drain-rate (* (sqrt height) drain-rate))
	   (drain-volume-increment (* current-drain-rate dt))
	   (volume-delta (- pump-volume-increment drain-volume-increment))
	   (new-volume (+ volume volume-delta))
	   (new-height (max 0 (/ new-volume area))))
      (update-values height volume pump-rate)
      ;; return an alist of interesting values
      `((height ,height) (volume ,volume) (pump-rate ,pump-rate) (drain-rate ,current-drain-rate))
      )))

(defmethod parameters-of-interest ((tank water-tank-plant-model))
  '((height "Height (m)" number) (area "Area (m^2)" number) (pump-rate-scale "Pump Scale (m^3 / s / mV)" number) (drain-rate "Drain Rate Factor (m^(5/2) / s)" number)))
 
(defmethod signals-of-interest ((tank water-tank-plant-model))
  '(height volume pump-rate drain-rate))

(defmethod plant-output-name ((tank water-tank-plant-model)) 'height)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; The original abstract plant model
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defvar *B* 1)
(defvar *K* 1)

(defclass abstract-plant-model ()
  ((state-1 :initform 0 :accessor state-1)
   (state-3 :initform 0 :accessor state-3))
  )

(defmethod initial-output ((pm abstract-plant-model))
  10.)
			   

(defmethod respond-to-command ((pm abstract-plant-model) command dt)
  ;; State-1 is the integral of PID
  ;; State-2 is the average value of State-1 in the last time step
  ;; State-3 is the running-sum of State-2 i.e. the Integral of the Integral of PID
  ;; Output is the average value of State-3 in the last time step
  ;; Feedback is a weighted average of State-2 and output
  (with-slots (state-1 state-3) pm 
    (let* ((new-state-1 (+ state-1 command)) ;; sum PID term to calculate the first integration
	   (state-2 (integral-slice state-1 dt)) ;;output after the first integrator
	   (new-state-3 (+ state-3 state-2))
	   (output (integral-slice state-3 dt))
	   (feedBack (+ (* state-2 *B*) (* Output *K*))))
      (update-values state-1 state-3)
      ;; return an alist of values
      `((feedback ,feedback) (output ,output) (state-2 ,state-2) (state-1 ,state-1)))))

;;; To do: to hook this up to graphical display need to have
;;; parameters-of-interest and signals-of-interest methods.

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; An Application Frame
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defparameter *color-alist* `(("red" . ,clim:+red+) ("blue" . ,clim:+blue+) ("green" . ,clim:+green+) ("orange" . ,+orange+) ("violet" . ,+violet+)))
(defparameter *color-list* (list clim:+blue+ clim:+red+ clim:+green+ +orange+ +violet+))

(clim:define-application-frame Control-System ()
  ((controller :accessor controller :initarg :controller)
   (plant :accessor plant :initarg :plant)
   (x-grid-increment :initform 1 :accessor x-grid-increment)
   (y-grid-increment :initform 1 :accessor y-grid-increment)
   (x-scale :initform 30 :accessor x-scale)
   (y-scale :initform 30 :accessor y-scale)
   (x-value-increment :initform .01 :accessor x-value-increment)
   (colors :initform *color-list* :accessor colors)
   (signal-names :initform '(height sensor error command pump-rate) :accessor signal-names)
   (plot-data :initform nil :accessor plot-data)
   (simulation-time :initform 20 :accessor simulation-time)
   (delta-time :initform .5 :accessor delta-time)
   (signal-distorter :initform nil :accessor signal-distorter)
   (plant-output-name :accessor plant-output-name)
   (awdrat-information :accessor awdrat-information :initform nil)
   (awdrat-enabled? :accessor awdrat-enabled? :initform nil)
   )
  (:top-level (control-system-top-level))
  (:command-table (control-system :inherit-from (clim:accept-values-pane)))
  (:menu-bar nil)
  (:panes
   (graph :application
	  :label "Simulation Results"
	  :display-after-commands t
	  :incremental-redisplay nil
	  :display-function 'display-tank
	  :scroll-bars t)
   (distorter-parameters :accept-values
			 :display-function '(clim:accept-values-pane-displayer :displayer signal-distorter-acceptor)
			 :scroll-bars :both)
   (controller-parameters :accept-values
			  :display-function '(clim:accept-values-pane-displayer :displayer controller-parameters-acceptor)
			  :scroll-bars t)
   (plant-parameters :accept-values
		     :display-function '(clim:accept-values-pane-displayer :displayer plant-parameters-acceptor)
		     :scroll-bars t)
   (display-parameters :accept-values
		       :display-function '(clim:accept-values-pane-displayer :displayer display-parameters-acceptor)
		       :scroll-bars t)
   (simulation-parameters :accept-values
		       :display-function '(clim:accept-values-pane-displayer :displayer simulation-parameters-acceptor)
		       :scroll-bars t)
   ;; (pointer-doc :pointer-documentation :scroll-bars nil :borders t :max-height '(2 :line) :height '(2 :line))
   (menu :command-menu 
	 :height :compute
	 :display-function '(clim:display-command-menu
			     :x-spacing 30
			     :row-wise t
			     :n-columns 7)
	 :max-height '(6 :line)
	 :height '(4 :line)
	 :scroll-bars t
	 :borders t)
   (interactor :interactor
               :borders t
               :scroll-bars :vertical
	       :height '(5 :line)
               :max-height '(5 :line)))
  (:layouts
   (normal 
    (clim:vertically ()
      (:fill
       (clim:horizontally ()
	 (.5 (clim:vertically ()
	       (.25 controller-parameters)
	       (.20 plant-parameters)
	       (.25 display-parameters)
	       (.15 simulation-parameters)
	       (.15 distorter-parameters)))
	 (:fill graph)))
      menu
      ;; pointer-doc
      interactor))))

(defmethod initialize-instance :after ((frame control-system) &rest args)
  (declare (ignore args))
  (setf (signal-names frame) (signals-of-interest (plant frame)))
  (setf (plant-output-name frame) (plant-output-name (plant frame)))
  )

(defmethod predicted-signal-name ((frame control-system))
  (string-upcase (concatenate 'string "Predicted-" (string (plant-output-name (plant frame))))))

(defmethod display-tank ((frame control-system) stream)
  (with-slots (plot-data x-grid-increment y-grid-increment x-scale y-scale x-value-increment colors signal-names awdrat-enabled? awdrat-information) frame
    (when plot-data
      (let* ((all-signal-names (append signal-names '(sensor error command)))
	     (Plot-data (loop for name in all-signal-names 
			    for data = (second (assoc name plot-data))
			    collect data)))
	(when (and awdrat-information awdrat-enabled?)
	  (let ((predicted-values (predicted-sensor-sequence (awdrat-information frame)))
		(predicted-signal-name (predicted-signal-name frame)))
	    (push predicted-values plot-data)
	    (push predicted-signal-name all-signal-names)))
	(plot plot-data
	      :stream stream
	      :x-grid-increment x-grid-increment 
	      :y-grid-increment y-grid-increment
	      :x-scale x-scale
	      :y-scale y-scale
	      :x-value-increment x-value-increment
	      :colors colors
	      :parameter-names all-signal-names
	      :error-information (when (and awdrat-enabled? awdrat-information) (error-detected awdrat-information))
	      ))))
  (values))


(defmacro accept-a-parameter (object parameter-name prompt ptype stream &rest options)
  `(progn
     (setf (,parameter-name ,object)
       (clim:accept ,ptype :stream ,stream :prompt ,prompt :default (,parameter-name ,object) ,@options))
     (terpri ,stream)
     ))

;;; This panel controls the display parameters
(defmethod display-parameters-acceptor ((frame control-system) stream)
  (clim:with-output-centered (stream)
    (clim:with-text-style (stream '(:fix :bold :large))
      (write-string "Display Parameters" stream)))
  (terpri stream)
  (terpri stream)
  (accept-a-parameter frame x-grid-increment "X Grid Increment" 'number stream)
  (accept-a-parameter frame y-grid-increment "Y Grid Increment" 'number stream)
  (accept-a-parameter frame x-scale "X Scale" 'number stream)
  (accept-a-parameter frame y-scale "Y Scale" 'number stream)
  ;; This really shouldn't be settable by the user.  It's the delta-t from the last simulation
  ;; (accept-a-parameter frame x-value-increment "X Value Increment" 'number stream)
  (accept-a-parameter frame signal-names "Signals to Plot" `(clim:subset-sequence ,(signals-of-interest (plant frame))) stream :view clim:+textual-view+)
  )

(defmethod simulation-parameters-acceptor ((frame control-system) stream)
  (clim:with-output-centered (stream)
    (clim:with-text-style (stream '(:fix :bold :large))
      (write-string "Simulation Parameters" stream)))
  (terpri stream)
  (terpri stream)
  (accept-a-parameter frame simulation-time "Simulation Time (s)" 'number stream)
  (accept-a-parameter frame delta-time "Time Step (s)" 'number stream)
  )

;;; This pane controls the parameters governing the PID controller
(defmethod controller-parameters-acceptor ((frame control-system) stream)
  (clim:with-output-centered (stream)
    (clim:with-text-style (stream '(:fix :bold :large))
      (write-string "Controller Parameters" stream)))
  (terpri stream)
  (terpri stream)
  (let ((controller (controller frame)))
    (accept-a-parameter controller ki "Integral Weight" 'number stream)
    (accept-a-parameter controller kd "Derivative Weight" 'number stream)
    (accept-a-parameter controller kp "Proportionality Weight" 'number stream)
    (accept-a-parameter controller setpoint "Set Point (m)" 'number stream)
    (accept-a-parameter controller lower-limit "Lower Limit" 'number stream)
    (accept-a-parameter controller upper-limit "Upper Limit" 'number stream)
    ))

;;; This pane controls the parameters governing the plant
;;; The plant's parameter-of-interest method provides a list
;;; of (parameter-names prompt-string presentation-type) for each
;;; parameter of interest.  Doing the setf of the frames parameter
;;; requires a particularly obscure incantation below
(defmethod plant-parameters-acceptor ((frame control-system) stream)
  (clim:with-output-centered (stream)
    (clim:with-text-style (stream '(:fix :bold :large))
      (write-string "Plant Parameters" stream)))
  (terpri stream)
  (terpri stream)
  (let* ((plant (plant frame))
	 (parameters-of-interest (parameters-of-interest plant)))
    (loop for (name string ptype) in parameters-of-interest
	do (funcall (fdefinition `(setf ,name))
		    ;; Is that the ugliest thing you've ever seen
		    (clim:accept ptype :stream stream :prompt string :default (funcall name plant))
		    plant)
	   (terpri stream))))

(clim:define-presentation-type percent ()
  :inherit-from '(number))

(clim:define-presentation-method clim:presentation-typep (object (type percent))
  (numberp object))

(clim:define-presentation-method clim:accept  ((type percent) stream (view clim:textual-view) &key)
  (declare (ignorable stream))
  (let ((typed-value (call-next-method)))
    (/ typed-value 100.0)))

(clim:define-presentation-method clim:present  (object (type percent) stream (view clim:textual-view)
						       &key )
  (clim:present  (* object 100) 'number :stream stream :view view))

(defmethod signal-distorter-acceptor ((frame control-system) stream)
  (clim:with-output-centered (stream)
    (clim:with-text-style (stream '(:fix :bold :large))
      (write-string "Signal Corruptors" stream)))
  (terpri stream) 
  (terpri stream)
  (let* ((current-distorter (signal-distorter frame))
	 (current-distorter-description (first current-distorter))
	 (current-distorter-type (first current-distorter-description))
	 (type (clim:accept `(clim:member-alist (("None" . identity) 
						 ("Fixed Offset" . fixed-offset) 
						 ("Increasing Offset" . increasing-offset)
						 ("Stepped Percent" . stepped-percent)))
			    :stream stream
			    :view clim:+textual-view+
			    :prompt "Type of signal distortion"
			    :default (or current-distorter-type 'identity))))
    (terpri stream)
    (case type
      (identity (setf (signal-distorter frame) `((identity) ,#'(lambda (x time) (declare (ignore time)) x))))
      (fixed-offset 
       (let ((offset (if (eql current-distorter-type 'fixed-offset)
			 (second current-distorter-description)
		       (default-parameters 'fixed-offset-distorter))))
	 (setq offset (clim:accept 'number
				   :stream stream
				   :prompt "Amount of distortion"
				   :default offset))
	 (setf (signal-distorter frame) `((fixed-offset ,offset) ,(make-a-fixed-offset-distorter offset)))))
      (increasing-offset
	  (let ((parameters (if (eql current-distorter-type 'increasing-offset)
				(rest current-distorter-description)
			      (default-parameters 'increasing-offset-distorter))))
	    (destructuring-bind (start-time end-time max-distortion) parameters
	      (setq start-time (clim:accept 'number
					    :stream stream
					    :prompt "Time to start distortion"
					    :default start-time))
	      (terpri stream)
	      (setq end-time (clim:accept 'number
					  :stream stream
					  :prompt "Time to stop increasing offset"
					  :default end-time))
	      (terpri stream)
	      (setq max-distortion (clim:accept 'number
						:stream stream
						:prompt "Maximum amount of distortion"
						:default max-distortion))
	      (setf (signal-distorter frame) `((increasing-offset ,start-time ,end-time ,max-distortion)
					       ,(make-an-increasing-offset-distorter start-time end-time max-distortion))))))
      (stepped-percent
       (let ((parameters (if (eql current-distorter-type 'stepped-percent)
			     (rest current-distorter-description)
			   (default-parameters 'stepped-percent-distorter))))
	 (destructuring-bind (initial-percent delta-percent sample-rate) parameters
	   (setq initial-percent (clim:accept 'percent
					      :stream stream
					      :prompt "Initial percent offset"
					      :default initial-percent))
	   (terpri stream)
	   (setq delta-percent (clim:accept 'percent
					    :stream stream
					    :prompt "Amount to increase percent offset by"
					    :default delta-percent))
	   (terpri stream)
	   (setq sample-rate (clim:accept 'number
					  :stream stream 
					  :prompt "Number of samples after which to increment offset"
					  :default sample-rate))
	   (setf (signal-distorter frame) `((stepped-percent ,initial-percent ,delta-percent ,sample-rate)
					    ,(make-a-staged-percent-distorter initial-percent delta-percent sample-rate)))))))))

(defmethod default-parameters ((type (eql 'fixed-offset-distorter))) 2)
(defmethod default-parameters ((type (eql 'stepped-percent-distorter))) `(2 .01 10))
(defmethod default-parameters ((type (eql 'increasing-offset-distorter))) '(3 10 4))

(defvar *control-system-frame* nil)

(defmethod control-system-top-level ((frame control-system ) &REST OPTIONS)
  ;; Note that you need to arrange to kill this process after
  ;; the editor-dies.  That's in the frame-exit method below
  (let ((*package* (find-package (string-upcase "awdrat")))
	(*debugger-hook* #'debugger-hook))
    (APPLY #'clim:default-frame-top-level
	   frame
	   :prompt ">"
	   OPTIONS)))

(defun debugger-hook (condition hook)
  (declare (ignore hook))
  (let* ((*application-frame* *control-system-frame*)
	 (*error-output* (clim:frame-standard-output *application-frame*))
	 (stream (clim:get-frame-pane *application-frame* 'interactor)))
    (clim:stream-close-text-output-record stream)
    (clim-utils:letf-globally
        (((clim:stream-current-output-record stream) (clim:stream-output-history stream))
	 ((clim:stream-recording-p stream) t)
	 ((clim:stream-drawing-p stream) t)
	 ((clim-internals::stream-current-redisplay-record stream) nil))
      (setf (clim:command-menu-enabled 'clim-env:listener-restarts *application-frame*) t)
      (clim-env:enter-debugger condition stream :own-frame t ))))

;; This doesn't work because the lisp listener is still doing its thing
;; you could probably use the design-editor's windows however
(defun run-control-system ()
  (mp:process-run-function
	"Control System Analyzer"
    #'(lambda ()
	(let ((width 1000)
	      (height 750))
	  (setq *control-system-frame* (clim:make-application-frame 'control-system
								    :pretty-name "control system analyzer"
								    :controller (make-instance 'pid-controller)
								    :plant (make-instance 'water-tank-plant-model)
							     :width width
							     :height height)))
	  (clim:run-frame-top-level *control-system-frame*))
    ))

(clim-env::define-lisp-listener-command (com-start-control-system :name t)
    ()
  (run-control-system))

(define-condition observed-sensor-value-seems-wrong (error) 
  ((simulated-time :accessor simulated-time :initarg :simulated-time)
   (predicted-value :accessor predicted-value :initarg :predicted-value)
   (sensor-value :accessor sensor-value :initarg :sensor-value)
   )
  (:report (lambda (self stream)
	     (format stream "The reported sensor value ~a differs from the predicted-value ~a at time ~a"
		     (sensor-value self)
		     (predicted-value self)
		     (simulated-time self)))))
  
(defmethod simulate ((frame control-system))
  (with-slots (plant controller simulation-time delta-time signal-distorter signal-names plant-output-name plot-data awdrat-enabled? awdrat-data) frame
    (setq plot-data
      (do-a-simulation plant controller
		       simulation-time delta-time
		       (second signal-distorter)
		       (signals-of-interest plant)
		       plant-output-name
		       ))
    (setf (x-value-increment frame) delta-time)
    ;; Pre-compute the x and y scale
    (let* ((pane (clim:get-frame-pane frame 'graph))
	   (width (clim:window-inside-width pane))
	   (height (clim:window-inside-height pane))
	   (all-plotted-signals (append '(error signal command) signal-names))
	   (graph-pane (clim:get-frame-pane frame 'graph))
	   (signal-names-width (loop for name in (if awdrat-enabled? (cons (predicted-signal-name frame) all-plotted-signals) all-plotted-signals)
				   maximize (multiple-value-bind (width height) (clim:text-size graph-pane (string name))
					      (declare (ignore height))
					      ;; (format *error-output* "~%string ~a width ~a" (string name) width)
					      (+ width 40))))
	   (next-grid-after-simulated-time (* (ceiling (* delta-time (length (second (first plot-data)))) (x-grid-increment frame)) (x-grid-increment frame)))
	   (min-y nil)
	   (max-y nil))
      ;; (format *error-output* "~%Width ~a Names width ~a" width signal-names-width)
      (loop for (name data) in plot-data
	  when (member name all-plotted-signals)
	  do (loop for value across data
		 when (or (null max-y) (> value max-y))
		 do (setq max-y value)
		 when (or (null min-y) (< value min-y))
		 do (setq min-y value)))
      (setq max-y (* (ceiling max-y (y-grid-increment frame)) (y-grid-increment frame))
	    min-y (* (floor min-y (y-grid-increment frame)) (y-grid-increment frame)))
      ;; The value in x-scale below is the "left-margin" in the plot code in setting up its transform
      (let ((x-scale (/ (float (- width signal-names-width 40)) next-grid-after-simulated-time))
	    (y-scale (/ (float (- height 100)) (- max-y min-y))))
	;; (format *error-output* "~%Scaled last-grid ~a ~a" (* x-scale next-grid-after-simulated-time) x-scale)
	(setf (x-scale frame) x-scale
	      (y-scale frame) y-scale)))
    ))

(define-control-system-command (com-simulate :menu t :name t)
    (&key (Initial-value 'number))
  (when initial-value
    (let* ((plant (plant clim:*application-frame*))
	   (output-name (plant-output-name plant)))
      ;; pass the initial state into the plant
      (funcall (fdefinition `(setf ,output-name)) initial-value plant)))
  (unwind-protect
      (progn (when (awdrat-enabled? clim:*application-frame*)
	       (install-all-events))
	     (simulate clim:*application-frame*))
    (when (awdrat-enabled? clim:*application-frame*)
      (uninstall-all-events))))

(define-control-system-command (com-enable-armet :menu t :name t)
    ((enabled? 'boolean :default nil :prompt "Enable ARMET Monitoring?")) ; Tasos, changed AWDRAT to ARMET
  (setf (awdrat-enabled? clim:*application-frame*) enabled?
	(awdrat-information clim:*application-frame*) nil))
