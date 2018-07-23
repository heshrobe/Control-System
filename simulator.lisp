;;; -*- Mode: Commonlisp; package: Awdrat; Readtable: Joshua -*-

(in-package :awdrat)

;;;(eval-when (:compile-toplevel :load-toplevel)
;;;  (export 'clim::with-output-centered 'clim))

;;; This is in clim-fixes now, no need for it here.
;;;;;; for some reason, emacs formats this correctly but not centering-output
;;;(defmacro clim-internals::with-output-centered ((stream &key (move-cursor t)) &body body)
;;;  (clim-internals::default-output-stream stream centering-output)
;;;  `(flet ((centering-output-body (,stream) ,@body))
;;;     (declare (dynamic-extent #'centering-output-body))
;;;     (clim-internals::invoke-centering-output
;;;      ,stream #'centering-output-body 
;;;      :move-cursor ,move-cursor)))

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

(defmacro square (form)
  (let ((variable (gensym)))
    `(let ((,variable ,form))
       (* ,variable ,variable))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Definition of a generic parameterizable PID controller
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass pid-controller ()
  (
   ;; This is needed in order to calculate derivative
   (signal-error :initform 0 :accessor signal-error)
   ;; This isn't really needed, nothing uses it
   (proportional :initform 0 :accessor proportional)
   ;; ditto
   (derivative :initform 0 :accessor derivative)
   ;; This is needed because it's accumulated over time
   (integral :initform 0 :accessor integral)
   (kp :initarg :kp :accessor kp :initform 2.0)
   (ki :initarg :ki :accessor ki :initform 0.1)
   (kd :initarg :kd :accessor kd :initform 0.01)
   (setpoint :initarg :setpoint :accessor setpoint :initform 10)
   (lower-limit :initarg :lower-limit :accessor lower-limit :initform nil)
   (upper-limit :initarg :upper-limit :accessor upper-limit :initform nil)
   (saved-kp :accessor saved-kp)
   (saved-ki :accessor saved-ki)
   (saved-kd :accessor saved-kd) 
   ;; This is the guy used to estimate the
   ;; state given the control provided and the observed outputs
   (kalman-filter :accessor kalman-filter :initarg :kalman-filter)
   ))

(defmethod reset ((controller pid-controller))
  (setf (integral controller) 0
	(signal-error controller) 0
	(proportional controller) 0
	(derivative controller) 0
	))

(defmethod save-pid ((controller pid-controller))
  (setf (saved-kp controller) (kp controller)
	(saved-ki controller) (ki controller)
	(saved-kd controller) (kd controller)))

(defmethod restore-pid ((controller pid-controller))
    (setf (kp controller) (saved-kp controller)
	  (ki controller) (saved-ki controller)
	  (kd controller) (saved-kd controller))
  )
  
;;; I rewrote this so that it makes explicit the computational steps that are required.
;;; Now the model of this can be interesting.
;;; Also it's a function now so that I can wrap its internals
(defun respond-to-sensor-value (controller last-control observation dt)
  (with-slots (signal-error integral kp ki kd setpoint upper-limit lower-limit kalman-filter) controller
    (flet ((estimate-state (last-control observation) 
	     (trace-format "~%Respond to sensor observation ~a last control ~a" observation last-control)
	     (estimate-state kalman-filter last-control observation))
	   (compute-error (state-estimate) (- state-estimate setpoint))
	   (compute-new-derivative (new-signal-error) (derivative-slice signal-error dt))
	   (compute-new-integral (new-signal-error) 
	     (trace-format "~%In simulator old-signal-error ~a new-signal-error ~a dt ~a" signal-error new-signal-error dt)
	     (let ((answer (integral-slice signal-error dt)))
	       (trace-format "~%In simulator integral is ~a" answer)
	       answer))
	   (accumulate-error (new-integral) (incf integral new-integral))
	   (update-state (new-signal-error) (update-values signal-error))
	   (compute-new-proportional-term (proportional) (* kp proportional))
	   (compute-new-integral-term (integral) 
	     (trace-format "~%In simulator compute-new-integral-term integral is ~a" integral)
	     (* ki integral))
	   (compute-new-derivative-term (derivative) (* kd derivative))
	   (compute-total-correction (p i d) (- (+ p i d)))
	   (clip (value)
	     (when lower-limit (setq value (max lower-limit value)))
	     (when upper-limit (setq value (min upper-limit value)))
	     value))
      (multiple-value-bind (estimated-state prior-state-estimate estimated-output residual) (estimate-state last-control observation)
	(declare (ignore prior-state-estimate estimated-output))
      (trace-format "~%Controller estimated state ~d" estimated-state)
	(let* ((new-signal-error (compute-error estimated-state)))
	  (let ((new-proportional new-signal-error)
		(new-derivative (compute-new-derivative new-signal-error))
		(new-integral (compute-new-integral new-signal-error)))
	    (accumulate-error new-integral)
	    (setf (derivative controller) new-derivative
		  (proportional controller) new-proportional)
	    (update-state new-signal-error)
	    (let ((prop-term (compute-new-proportional-term new-proportional))
		  (int-term (compute-new-integral-term new-integral))
		  (deriv-term (compute-new-derivative-term new-derivative)))
	      (let ((correction (compute-total-correction prop-term int-term deriv-term)))
		(let ((clipped-correction (clip correction)))
		  (values clipped-correction signal-error residual))))))))))

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
(defun do-a-simulation (plant controller time dt signal-distortion-pair signal-labels plant-output-name &key (display t))
  ;; reset the running sum of the integral in the PID controller
  ;; need to catch this in the monitor
  (reset controller)
  ;; need to catch this in the monitor?
  (reset plant)
  (let* ((number-of-steps (ceiling time dt))
	 (signal-distorter-form (second signal-distortion-pair))
	 (signal-distorter nil)
	 (signal-distarter-type (first (first signal-distortion-pair)))
	 (error-sequence (make-array number-of-steps :fill-pointer 0 :adjustable t))
	 (spoofed-sensor-sequence (make-array number-of-steps :fill-pointer 0 :adjustable t))
	 (residual-sequence (make-array number-of-steps :fill-pointer 0 :adjustable t))
	 (command-sequence (make-array number-of-steps :fill-pointer 0 :adjustable t))
	 (plant-signal-alist  (loop for name in signal-labels collect (list name (make-array number-of-steps :fill-pointer 0 :adjustable t))))
	 (full-signal-alist (append `((error ,error-sequence) 
				      (residual ,residual-sequence)
				      ,@(if (eql signal-distarter-type 'identity) nil `((spoofed-sensor ,spoofed-sensor-sequence)))
				      (command ,command-sequence))
				    plant-signal-alist))
	 ;; initialize the plant before making the kalman-filter for the tank
	 ;; (initial-output (initial-output plant))
	 ;; (initial-alist (initialize plant initial-output))
	 (signal-hacked nil))
    ;; provide the controller with its kalman-filter
    ;; Note: need to catch this in the monitor
    (setf (kalman-filter controller) (funcall (kalman-filter-constructor plant) plant dt))
    ;; Because this might depend on the kalman filter being initialized
    (setq signal-distorter (apply (first signal-distorter-form) (rest signal-distorter-form)))
    ;; The simulation will start by initializing the plant on time step 0 and geting its outputs
    ;; and then calling the controller on time step 0.  
    ;; Then on each succeeding time step we pass the controller command
    ;; to the plant and get its output.
    (let* ((frame clim:*application-frame*)
	   (controller-hacker (controller-hacker frame))
	   (pane (clim:get-frame-pane frame 'graph))
	   (pane-height (clim:window-inside-height pane))
	   (pane-width (clim:window-inside-width pane))
	   (dashed-line-style (clim:make-line-style :dashes t))
	   (set-point (setpoint controller))
	   (transform (clim:make-scaling-transformation (/ pane-width 20) (/ pane-height (* 2 set-point))))
	   (offset (clim:untransform-distance transform 3 0))
	   )
       (clim:with-first-quadrant-coordinates (pane 0 pane-height)
	(clim:with-drawing-options (pane :transformation transform)
	  (clim:with-output-recording-options (pane :draw t :record nil)
	    (when display 
	      (clim:window-clear pane)
	      ;; line for the set-point
	      (clim:draw-line* pane 0 set-point pane-width set-point :ink clim:+black+ :line-style dashed-line-style)
	      (clim:draw-line* pane 0 (* 1.5 set-point) pane-width (* 1.5 set-point) :ink clim:+red+ :line-style dashed-line-style)
	      (clim:draw-line* pane 0 (* .5 set-point) pane-width (* .5 set-point) :ink clim:+red+ :line-style dashed-line-style)
	      ;; lines for the tank
	      (clim:draw-lines* pane (list (- 5 offset) (* 2 set-point) (- 5 offset) 0 (+ 10 offset) 0 (+ 10 offset) (* 2 set-point)) 
				:ink clim:+black+ :line-thickness 3))
	    (catch 'stop-the-simulation
	      (loop for sim-time from 0 by dt below time
		  for command = 0 then next-command
		  for plant-output = (let ((plant-observation-alist (respond-to-command plant command dt)))
				       ;; now observe everything the plant told us
				       (loop for (name vector) in plant-signal-alist
					   for observation = (second (assoc name plant-observation-alist))
					   do (vector-push-extend observation vector))
				       ;; and then return the plant's output from the alist
				       (second (assoc plant-output-name plant-observation-alist)))
		  for spoofed-sensor = (if signal-distorter (funcall signal-distorter (setpoint controller) plant-output command sim-time) plant-output)
		  for next-command = (multiple-value-bind (command error residual) (respond-to-sensor-value controller command spoofed-sensor dt)
				       ;; and observe the sensor-value, error, and controller's command
				       (observe error spoofed-sensor command residual)
				       command)
		  ;; do (trace-format "~%Actual plant output at time ~d is ~a" sim-time plant-output)
		  when display
		  do (clim:draw-rectangle* pane 5 0 10 plant-output :ink +light-blue+)
		     (sleep .1)
		     (clim:draw-rectangle* pane 5 0 10 plant-output :ink clim:+background-ink+)
		  when (and controller-hacker (not signal-hacked) (first controller-hacker) (>= sim-time (second controller-hacker)))
		  do (setq signal-hacked t)
		     (case (first controller-hacker)
		       (kp (setf (kp controller) (third controller-hacker)))
		       (ki (setf (ki controller) (third controller-hacker)))
		       (kd (setf (kd controller) (third controller-hacker)))
		       )
		     ;; the below will be irrelevant now that the tracers throw to stop-the-simulation
		     ;; I think.
		  until (and (awdrat-enabled? clim:*application-frame*)
			     (or (error-detected (awdrat-information clim:*application-frame*))
				 (miscalculation (awdrat-information clim:*application-frame*))))
			))
	    ))))
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
	  :x-grid-increment 5 :y-grid-increment 5
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

;;; The constructor gets called with the parameters of interest plus the application frame in case it wants to gobble data out of that
;;; A distorter gets called with the setpoint of the controller, the command issued by the controller, the plant-output and the simulation-time

(defun make-an-identity-distorter ()
  (flet ((distorter (set-point observation command time)
	   (declare (ignore set-point command time))
	   observation))
    #'distorter))

(defun make-a-fixed-offset-distorter (start-time stop-time offset &optional control-system)
  (declare (ignore control-system))
  (flet ((distorter (set-point observation command time) 
	   (declare (ignore set-point command))
	   (cond
	    ((< time start-time) observation)
	    ((<= start-time time stop-time)
	     (- observation offset))
	    ((> time stop-time) observation))))
    #'distorter))

(defun make-a-setpoint-offset-distorter (start-time stop-time offset &optional control-system)
  (declare (ignore control-system))
  (flet ((distorter (set-point observation command time) 
	   (declare (ignore command))
	   (cond
	    ((< time start-time) observation)
	    ((<= start-time time stop-time)
	     (- set-point offset))
	    ((> time stop-time) observation))))
    #'distorter))

(defun make-an-increasing-offset-distorter (start-time stop-time total-amount &optional control-system)
  (declare (ignore control-system))
  (let ((start-time start-time)
	(slope (/ (float total-amount) (- stop-time start-time)))
	(stop-time stop-time))
    (flet ((distorter (set-point observation command time)
	     (declare (ignore set-point command))
	     (cond
	      ((< time start-time) observation)
	      ((<= start-time time stop-time)
	       (- observation (* slope (- time start-time))))
	      ((> time stop-time)
	       (- observation (* slope (- stop-time start-time)))))))
      #'distorter)))

(defun make-a-staged-percent-distorter (initial-percent delta-percent sample-rate &optional control-system)
  (declare (ignore control-system))
  (let ((percent initial-percent))
    (flet ((distorter (set-point observation command time)
	     (declare (ignore set-point command))
	     (when (= time 0)
	       (setq percent initial-percent))
	     (when (zerop (mod time sample-rate))
	       (incf percent delta-percent))
	     (- observation (* percent observation))))
      #'distorter)))

#|
The attacker is restricted to a residual smaller than the threshold.
The residual is the difference between the a-priori prediction of the output value  and the observed output value.
The attacker can inject "noise" such that the real observed output value + noise = a-priori-prediction + allowed-residual

let Y be the observed output
     Y' be the predicted output
     R be the allowed residual
     N be the injected noise
     Y'' is the faked output

     R = Y'' - Y'
     Y'' = Y' + R

     N = Y'' - Y

|#

;;; (defun make-kalman-filter-for-tank (water-tank dt &optional (initial-variance 0))
(defun make-an-optimal-kalman-distorter (threshold direction control-system)
  (let* ((dt (delta-time control-system))
	 (plant (plant control-system))
	 (kalman-filter (funcall (kalman-filter-constructor plant) plant dt))
	 (max-residual (calculate-max-residual kalman-filter threshold)))
    (flet ((distorter (set-point observation command time)
	     (declare (ignore set-point time))
	     ;; you need to provide the KF with the distorted output not the observation, which it never sees.
	     ;; doing that requires calling the predict function first.
	     (multiple-value-bind (posterior-state-estimate prior-state-estimate estimated-output) (predictor kalman-filter command)
	       (declare (ignore posterior-state-estimate prior-state-estimate))
	       ;; We're going to calculate the injected distortion, just to model what's in the paper
	       (let* ((distorted-output (if (eql direction 'up) (+ estimated-output max-residual) (- estimated-output max-residual)))
		      (noise (- observation distorted-output)))
		 ;; now update the KF with the output it actually sees.
		 (estimate-state kalman-filter command distorted-output)
		 (values distorted-output noise)
	     ))))
     (values #'distorter kalman-filter))))


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
   (pump-rate-scale :accessor pump-rate-scale :initarg :pump-rate-scale :initform 10)
   ;; the proportionality constant for the drain-hole in cu-ft/sec/exp(ft,.5)
   (drain-rate :accessor drain-rate :initarg :drain-rate :initform 1)
   ;; Now the state-variables
   ;; the height of the water in the tank
   (height :accessor height :initform 10 :initarg :height)
   ;; the volume redundant with height, but who cares
   (volume :accessor volume)
   ;; the old pump rate
   (pump-rate :accessor pump-rate :initform 0)
   (sensor-noise-sigma :accessor sensor-noise-sigma :initform 0.2 :Initarg :sensor-noise-sigmma)
   (system-noise-sigma :accessor system-noise-sigma :initform 0.2 :initarg :system-noise-sigma)
   (simulation-frame :accessor simulation-frame :initarg :simulation-frame)
   ))

(defmethod reset ((tank water-tank-plant-model))
  (setf (height tank) (initial-tank-height (simulation-frame tank))
	(pump-rate tank) 0)  
  )

(defmethod kalman-filter-constructor ((tank water-tank-plant-model))
  'make-kalman-filter-for-tank)

(defmethod initialize-instance :after ((tank water-tank-plant-model) &rest args)
  (declare (ignore args))
  (setf (volume tank) (* (height tank) (area tank))))

(defmethod (setf height) :after (new-value (tank water-tank-plant-model))
  (setf (volume tank) (* new-value (area tank))))

(defmethod initialize ((tank water-tank-plant-model) initial-value)
  (with-slots (height pump-rate pump-rate-scale drain-rate volume area) tank
    ;; using setf so that the setf method calculates the volume
    (setf (height tank) initial-value
	  (pump-rate tank) 0)
    (let ((current-drain-rate (* (sqrt height) drain-rate)))
      `((height ,height) (sensor-value ,height) (volume ,volume) (pump-rate 0) (drain-rate ,current-drain-rate)))))

(defmethod initial-output ((tank water-tank-plant-model))
  (height tank))

(defmethod tank-drain ((tank water-tank-plant-model) dt)
  (with-slots (height drain-rate volume area) tank
    ;; notice this is using only the old value since we're calculating 
    ;; the new value using this 
    (let* ((current-drain-rate (* (If (minusp height) 0 (sqrt height)) drain-rate))
	   (drain-volume-increment (* current-drain-rate dt)))
      drain-volume-increment
      )))

(defmethod respond-to-command ((tank water-tank-plant-model) control dt)
  (with-slots (height pump-rate pump-rate-scale drain-rate volume area) tank
    (let* ((new-pump-rate (* pump-rate-scale control))
	   (pump-volume-increment (integral-slice pump-rate dt))
	   ;; notice this is using only the old value since we're calculating
	   ;; the new value using this
	   (current-drain-rate (if (minusp height) 0 (* (sqrt height) drain-rate)))
	   (drain-volume-increment (* current-drain-rate dt))
	   (volume-delta (- pump-volume-increment drain-volume-increment))
	   (plant-random (randist:random-normal 0 (system-noise-sigma tank)))
	   (random-volume-addition (* plant-random area))
	   (new-volume (+ volume volume-delta random-volume-addition))
	   (new-height (max 0 (/ new-volume area))))
      (trace-format "~2%Real Plant responding to command ~a, old-height ~a, new height ~a old-pump rate ~a pump rate ~a" 
		    control height new-height pump-rate new-pump-rate)
      (trace-format"~%Plant random ~a" plant-random)
      (update-values height volume pump-rate)
      ;; return an alist of interesting values
      (let* ((sensor-random (randist:random-normal 0 (system-noise-sigma tank)))
	     (noisy-height (+ height sensor-random)))
	(trace-format"~%Sensor random ~a" sensor-random)
	(trace-format "~%Sensed height ~a" noisy-height)
	`((height ,height) 
	  (sensor-value ,noisy-height)
	  (volume ,volume) (pump-rate ,pump-rate) (drain-rate ,current-drain-rate)
	  (pump-volume-increment ,pump-volume-increment)
	  (pump-rate-scale ,pump-rate-scale)))
      )))

(defmethod parameters-of-interest ((tank water-tank-plant-model))
  '((height "Height (m)" number)
    (sensor-noise-sigma "Sigma of sensor noise (m)" number)
    (system-noise-sigma "Sigma of system noise (m)" number)
    (area "Area (m^2)" number)
    (pump-rate-scale "Pump Scale (m^3 / s / mV)" number) 
    (drain-rate "Drain Rate Factor (m^(5/2) / s)" number)))
 
(defmethod signals-of-interest ((tank water-tank-plant-model))
  '(height sensor-value volume pump-rate drain-rate))

(defmethod plant-output-name ((tank water-tank-plant-model)) 'sensor-value)
(defmethod plant-state-name ((tank water-tank-plant-model)) 'height)

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
;;; A kalman filter for the water tank plant model
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Note: At creation time we are given all the parameters of the tank
(defclass water-tank-kalman-filter (extended-scalar-kalman-filter)
  ((dt :accessor dt :initarg :dt)
   (pump-rate :accessor pump-rate :initarg :pump-rate)
   (pump-rate-scale :accessor pump-rate-scale :initarg :pump-rate-scale)
   (drain-rate :accessor drain-rate :initarg :drain-rate)
   (area :accessor area :initarg :area))
  )

;;; Gobble all the relevant data from the physical plant.
;;; I'm assuming that the plant is initialized so its height
;;; is the right value, and unless otherwise told so, we know
;;; this with certainty so the initial-variance is 0.
(defun make-kalman-filter-for-tank (water-tank dt &optional (initial-variance 0))
  (make-instance 'water-tank-kalman-filter
    :dt dt
    :pump-rate (pump-rate water-tank)
    :pump-rate-scale (pump-rate-scale water-tank)
    :drain-rate (drain-rate water-tank)
    :area (area water-tank)
    ;; variance is square of signma
    :system-variance (square (system-noise-sigma water-tank))
    :observation-variance (square (sensor-noise-sigma water-tank))
    ;; Note: We could make this more interesting by making the state be the volume of water
    ;; and the sensor value by the height of the water column in the tank in which case
    ;; the observation gain would be the area.  Then the 
    ;; feedback control would need to be changed as well I think.
    :observation-gain 1
    :initial-state (height water-tank)
    :initial-variance initial-variance))

;;; This is the state equation evaluation:
;;; new-height = current-height + control-input * pump-rate-scale * dt  - sqrt(height) * drain-rate * dt

;;; Note: Height here is the previous state estimate from the filter not the height from the tank.
(defmethod prior-state-estimate ((filter water-tank-kalman-filter) control)
  (with-slots (pump-rate pump-rate-scale drain-rate area (height previous-state) dt) filter
    (let* ((volume (* area height))
	   (new-pump-rate (* pump-rate-scale control))
	   (pump-volume-increment (integral-slice pump-rate dt))
	   ;; notice this is using only the old value since we're calculating
	   ;; the new value using this
	   (current-drain-rate (if (minusp height) 0 (* (sqrt height) drain-rate)))
	   (drain-volume-increment (* current-drain-rate dt))
	   (volume-delta (- pump-volume-increment drain-volume-increment))
	   (new-volume (+ volume volume-delta))
	   (new-height (max 0 (/ new-volume area))))
      (trace-format "~%KF prediction given command ~a, current height ~a old pump rate ~a pump rate ~a" 
		    control height pump-rate new-pump-rate)
      (trace-format "~%KF Prior state estimated height ~a given ~a ~a ~a ~a"
		    height pump-volume-increment drain-volume-increment volume-delta new-height)
      (update-values pump-rate)
      new-height
      )))

;;; The derivate of the state equation is then:
;;; d(new-height)/dx = 1 + 0 - drain-rate * dt * 1/2 height^-1/2
;;;                  = 1 - (drain-rate * dt) / 2 * sqrt(height)
;;; notice that this doesn't depend on the control input term
;;; That's because it's independent of height, so it doesn't contribute
;;; to the variance of the new-height.
;;; Note: If the height is 0 or minus this blows up

(defmethod prior-variance-estimate ((filter water-tank-kalman-filter))
  (with-slots (dt (a system-gain) (Q system-variance) (previous-variance-estimate previous-variance) (height previous-state) drain-rate) filter
    (let* ((derivative (1- (/ (* drain-rate dt) (* 2 (sqrt height)))))
	   (estimate (+ (* derivative derivative previous-variance-estimate) Q)))
      (values estimate
	      ;; this is what you'd calculate for a normal kalman filter
	      ;; I just included it here to see the difference
	      ;; (+ (* a a previous-variance-estimate) Q)
	      ))))

;;; The requirement is that residual^2/sigma-r^2 < threshold.
;;; But Calculate-sigma-r returns not a standard deviation but the variance
;;; I.e. Sigma-r^2.
;;; residual^2 < sigma-r * threshold
;;; residual < sqrt (sigma-r * threshold)
;;; Note: double check all this
(defmethod calculate-max-residual ((filter water-tank-kalman-filter) threshold)
  (let ((sigma-r (calculate-steady-state-sigma-r filter)))
    (sqrt (* sigma-r threshold))
  ))

(defmethod calculate-current-sigma-r ((filter water-tank-kalman-filter) prior-variance-estimate)
  (with-slots ((R observation-variance) (h observation-gain)) filter
    (let ((sigma-p prior-variance-estimate))
      (+ (* h h sigma-p) R))))

(defmethod calculate-steady-state-sigma-r ((filter water-tank-kalman-filter))
  (with-slots ((R observation-variance) (h observation-gain)) filter
    (let ((sigma-p (steady-state-kalman-sigma filter)))
      (+ (* h h sigma-p) R))))

;;; Note: If I got this right, then Q needs to be at least 4 times R.
(defmethod steady-state-kalman-sigma ((filter water-tank-kalman-filter))
  (with-slots ((R observation-variance) (Q system-variance) ) filter
    (/ (+ Q (sqrt (+ (* Q Q) (* 4 Q R))))
       2)))

(defmethod check-residual ((filter water-tank-kalman-filter) residual threshold)
  (let* ((sigma-r (calculate-steady-state-sigma-r filter))
	 (ratio (/ (* residual residual) sigma-r)))
    (> ratio threshold)))
		  



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; An Application Frame
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defparameter *color-alist* `(("red" . ,clim:+red+) ("blue" . ,clim:+blue+) ("green" . ,clim:+green+) ("orange" . ,+orange+) ("violet" . ,+violet+)
						    ("forest-green" . ,+forest-green+)))
(defparameter *color-list* (list clim:+blue+ clim:+red+ clim:+green+ +orange+ +violet+ +forest-green+))

(clim:define-application-frame Control-System ()
  ((controller :accessor controller :initarg :controller)
   (plant :accessor plant :initarg :plant)
   (x-grid-increment :initform 5 :accessor x-grid-increment)
   (y-grid-increment :initform 5 :accessor y-grid-increment)
   (x-scale :initform 30 :accessor x-scale)
   (y-scale :initform 30 :accessor y-scale)
   (x-value-increment :initform .01 :accessor x-value-increment)
   (colors :initform *color-list* :accessor colors)
   (signal-names :initform '(height sensor-value) :accessor signal-names)
   (plot-data :initform nil :accessor plot-data)
   (initial-tank-height :initform 10 :accessor initial-tank-height)
   (simulation-time :initform 40 :accessor simulation-time)
   (delta-time :initform .5 :accessor delta-time)
   (signal-distorter :initform nil :accessor signal-distorter)
   (controller-hacker :initform (list nil 0 0) :accessor controller-hacker)
   (plant-output-name :accessor plant-output-name)
   (plant-state-name :accessor plant-state-name)
   (awdrat-information :accessor awdrat-information :initform nil)
   (awdrat-enabled? :accessor awdrat-enabled? :initform nil)
   (display-enabled? :accessor display-enabled? :Initform t)
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
   (plant-status :application
		 :label "System Status"
		 :display-after-commands t
		 :incremental-redisplay nil
		 :display-function 'display-status
		 :scroll-bars nil)		      
   (controller-attacks :accept-values
		       :display-function '(clim:accept-values-pane-displayer :displayer controller-hack-acceptor))
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
	       (.20 controller-parameters)
	       (.25 plant-parameters)
	       (.20 display-parameters)
	       (.15 simulation-parameters)
	       (.20 distorter-parameters)
	       ))
	 (:fill (clim:vertically ()
		  (.75 graph)
		  (.10 plant-status)
		  (.15 controller-attacks)))))
      menu
      ;; pointer-doc
      interactor))))

(defmethod initialize-instance :after ((frame control-system) &rest args)
  (declare (ignore args))
  (setf ;; (signal-names frame) (signals-of-interest (plant frame))
	(plant-output-name frame) (plant-output-name (plant frame))
	(plant-state-name frame) (plant-state-name (plant frame)))
  )

(defmethod predicted-signal-name ((frame control-system))
  (string-upcase (concatenate 'string "Predicted-" (string (plant-output-name (plant frame))))))

(defmethod display-status ((frame control-system) stream)
  (cond
   ((awdrat-enabled? frame)
    (let* ((monitor-information (awdrat-information frame))
	   (false-data-injected (when monitor-information (error-detected monitor-information)))
	   (parameter-over-written (when monitor-information (miscalculation monitor-information))))
      (cond
       (false-data-injected (write-string "False Data Injection Attack" stream))
       (parameter-over-written
	(destructuring-bind (parameter-name actual expected) parameter-over-written
	    (format stream "~%Controller parameter ~a Modified ~%Expected Control ~a ~%Actual Control ~a"
		    parameter-name actual expected)))
       (t (write-string "Normal" stream)))))
   (t (write-string "No monitoring" stream))))

(defmethod display-tank ((frame control-system) stream)
  (with-slots (plot-data x-grid-increment y-grid-increment x-scale y-scale x-value-increment colors signal-names awdrat-enabled? awdrat-information display-enabled?
	       signal-distorter) 
      frame
    (when (and plot-data display-enabled?)
      (let* ((all-signal-names (union signal-names (if (eql (first (first signal-distorter)) 'identity)
							'(error residual command)
							'(spoofed-sensor residual error command))))
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
  (accept-a-parameter frame initial-tank-height "Initial Water Level (m)" 'number stream)
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
    ;; (accept-a-parameter controller lower-limit "Lower Limit" 'number stream)
    ;; (accept-a-parameter controller upper-limit "Upper Limit" 'number stream)
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

(clim:define-presentation-type percent () :inherit-from '(number))

(clim:define-presentation-method clim:presentation-typep (object (type percent))
  (numberp object))

(clim:define-presentation-method clim:accept  ((type percent) stream (view clim:textual-view) &key)
  (declare (ignorable stream))
  (let ((typed-value (call-next-method)))
    (/ typed-value 100.0)))`
			
(clim:define-presentation-method clim:present  (object (type percent) stream (view clim:textual-view) &key)
	(clim:present (* object 100) 'number :stream stream :view view))

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
						 ("Set Point Offset" . setpoint-offset)
						 ("Increasing Offset" . increasing-offset)
						 ("Stepped Percent" . stepped-percent)
						 ("Optimal" . optimal)))
			    :stream stream
			    :view clim:+textual-view+
			    :prompt "Type of signal distortion"
			    :default (or current-distorter-type 'identity))))
    (terpri stream)
    (case type
      (identity 
       (setf (signal-distorter frame) `((identity) (make-an-identity-distorter))))
      (setpoint-offset
	  (let ((parameters (if (eql current-distorter-type 'setpoint-offset)
				(rest current-distorter-description)
			      (default-parameters 'setpoint-offset-distorter))))
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
	      (setf (signal-distorter frame) `((setpoint-offset ,start-time ,end-time ,max-distortion)
					       (make-a-setpoint-offset-distorter ,start-time ,end-time ,max-distortion))))))
      (fixed-offset
	  (let ((parameters (if (eql current-distorter-type 'fixed-offset)
				(rest current-distorter-description)
			      (default-parameters 'fixed-offset-distorter))))
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
	      (setf (signal-distorter frame) `((fixed-offset ,start-time ,end-time ,max-distortion)
					       (make-a-fixed-offset-distorter ,start-time ,end-time ,max-distortion))))))
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
					       (make-an-increasing-offset-distorter ,start-time ,end-time ,max-distortion))))))
      (stepped-percent
       (let ((parameters (if (eql current-distorter-type 'stepped-percent)
			     (rest current-distorter-description)
			   (default-parameters 'stepped-percent-distorter))))
	 (destructuring-bind (initial-percent delta-percent sample-rate) parameters
	   (setq initial-percent (clim:accept 'percent
					      :stream stream
					      :prompt "Initial offset (Number %), rep(Number) = Number/100"
					      :default initial-percent))
	   (terpri stream)
	   (setq delta-percent (clim:accept 'percent
					    :stream stream
					    :prompt "Amount to increase offset by (Number %), rep(Number) = Number/100"
					    :default delta-percent))
	   (terpri stream)
	   (setq sample-rate (clim:accept 'number
					  :stream stream 
					  :prompt "Number of samples after which to increment offset"
					  :default sample-rate))
	   (setf (signal-distorter frame) `((stepped-percent ,initial-percent ,delta-percent ,sample-rate)
					    (make-a-staged-percent-distorter ,initial-percent ,delta-percent ,sample-rate))))))
      (optimal 
       (let ((parameters (if (eql current-distorter-type 'optimal)
			     (rest current-distorter-description)
			   (default-parameters 'optimal-distorter))))
	 (destructuring-bind (threshold direction) parameters
	   (setq threshold (clim:accept 'number
					:stream stream
					:prompt "Threshold (number)"
					:default threshold))
	   (terpri stream)
	   (setq direction (clim:accept '(clim:member-alist (("Up" . up) 
							     ("Down" . down)))
					:view clim:+textual-view+
					:stream stream
					:prompt "Direction of Distortion (up or down)"
					:default direction))
	   (terpri stream)
	   (setf (signal-distorter frame) `((optimal ,threshold ,direction)
					    (make-an-optimal-kalman-distorter ,threshold ,direction ,clim:*application-frame*))))))
      )))

(defmethod default-parameters ((type (eql 'setpoint-offset-distorter))) '(0 20 2))
(defmethod default-parameters ((type (eql 'fixed-offset-distorter))) '(0 20 2))
(defmethod default-parameters ((type (eql 'stepped-percent-distorter))) `(0.2 0.02 10))
(defmethod default-parameters ((type (eql 'increasing-offset-distorter))) '(3 10 4))
(defmethod default-parameters ((type (eql 'optimal-distorter))) '(1.5 down))

(defmethod controller-hack-acceptor ((frame control-system) stream)
  (clim:with-output-centered (stream)
    (clim:with-text-style (stream '(:fix :bold :large))
      (write-string "Controller Corruptors" stream)))
  (terpri stream) 
  (terpri stream)
  (let* ((current-distorter (controller-hacker frame)))
    (setf (first current-distorter)
      (clim:accept `(clim:member-alist (("None" . nil) ("KP" . KP) ("KI" . KI) ("KD" . KD)))
		   :stream stream
		   :view clim:+textual-view+
		   :prompt "Controller Parameter "
		   :default (first current-distorter)))
    (when (first current-distorter)
      (terpri stream)
      (setf (second current-distorter)
	(clim:accept 'number
		     :stream stream
		     :prompt "Onset time"
		     :default (second current-distorter)))
      (terpri stream)
      (setf (third current-distorter)
	(clim:accept 'number
		     :stream stream
		     :prompt "New Value"
		     :default (third current-distorter))))
    ))

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

(defun run-control-system ()
  (mp:process-run-function
	"Control System Analyzer"
    #'(lambda ()
	(let ((width 1000)
	      (height 850))
	  (let* ((plant (make-instance 'water-tank-plant-model))
		 (controller (make-instance 'pid-controller)))
	    (setq *control-system-frame* (clim:make-application-frame 'control-system
								      :pretty-name "control system analyzer"
								      :plant plant
								      :controller controller
								      :width width
								      :height height))
	    (setf (simulation-frame plant) *control-system-frame*)))
	  (clim:run-frame-top-level *control-system-frame*))
    ))

(CLIM-env::define-lisp-listener-command (com-start-control-system :name t)
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
  (with-slots (plant controller simulation-time delta-time signal-distorter signal-names plant-output-name plot-data awdrat-enabled? display-enabled?) frame
    (save-pid controller)
    (unwind-protect 
	(setq plot-data
	  (do-a-simulation plant controller
			   simulation-time delta-time
			   signal-distorter
			   (signals-of-interest plant)
			   plant-output-name
			   :display display-enabled?
			   ))
      ;; Reset the control system parameters after the simulation, the attacker might have munged them
      (restore-pid controller))
    (setf (x-value-increment frame) delta-time)
    ;; Pre-compute the x and y scale
    (when display-enabled?
      (let* ((pane (clim:get-frame-pane frame 'graph))
	     (width (clim:window-inside-width pane))
	     (height (clim:window-inside-height pane))
	     (all-plotted-signals (union '(residual error signal command) signal-names))
	     (graph-pane (clim:get-frame-pane frame 'graph))
	     (signal-names-width (loop for name in (if awdrat-enabled? (cons (predicted-signal-name frame) all-plotted-signals) all-plotted-signals)
				     maximize (multiple-value-bind (width height) (clim:text-size graph-pane (string name))
						(declare (ignore height))
						;; (trace-format "~%string ~a width ~a" (string name) width)
						(+ width 40))))
	     (next-grid-after-simulated-time (max 1 (* (ceiling (* delta-time (length (second (first plot-data)))) (x-grid-increment frame))
						       (x-grid-increment frame))))
	     (min-y nil)
	     (max-y nil))
	;; (trace-format "~%Width ~a Names width ~a" width signal-names-width)
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
	  ;; (trace-format "~%Scaled last-grid ~a ~a" (* x-scale next-grid-after-simulated-time) x-scale)
	  (setf (x-scale frame) x-scale
		(y-scale frame) y-scale))))
    ))

(define-control-system-command (com-simulate :menu t :name t)
    (&key (Initial-value 'number))
  (when initial-value
    (let* ((plant (plant clim:*application-frame*))
	   (output-name (plant-state-name plant)))
      ;; pass the initial state into the plant
      (funcall (fdefinition `(setf ,output-name)) initial-value plant)))
  (unwind-protect
      (if (awdrat-enabled? clim:*application-frame*)
	  (with-harness (plant-startup)
	    ;; (install-all-events)
	    (simulate clim:*application-frame*))
	(simulate clim:*application-frame*))
    (when (awdrat-enabled? clim:*application-frame*)
      (uninstall-all-events))))

(define-control-system-command (com-enable-awdrat :menu t :name t)
    ((enabled? 'boolean :default nil :prompt "Enable AWDRAT Monitoring?")) ; Tasos, changed AWDRAT to ARMET
  (setf (awdrat-enabled? clim:*application-frame*) enabled?
	(awdrat-information clim:*application-frame*) nil))

(define-control-system-command (com-enable-display :name t :menu t)
    ((enabled? 'boolean :default nil :prompt "Enable Interactive Display"))
  (setf (display-enabled? clim:*application-frame*) enabled?)
  )
