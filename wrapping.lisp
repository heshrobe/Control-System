;;; -*- Mode: Commonlisp; package: Awdrat; Readtable: Joshua -*-

(in-package :awdrat)

;;; The plant events
;;; We just notice that the plant starts up
;;; The rest is done by tracersxx
(register-event plant-startup simulate)

(define-component-type plant-startup
    :top-level t
    :entry-events (plant-startup)
    :exit-events (plant-startup)
    :allowable-events (controller-step compute-proportional-term compute-integral-term compute-derivative-term)
    :inputs (frame)
    :outputs (plant-data)
    :components ()
    )

(defbehavior-model (plant-startup normal)
    :inputs (frame)
    :outputs (plant-data))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 
;;;      The controller Model
;;;      The conceptual steps of the computation:
;;; 
;;;                                               | sensor-value
;;;           --------------------------   estimate-error
;;;           |                              /           \
;;;           |                 compute-integral       compute-derivative
;;;           |                         |                        |
;;;     compute-proportional-term  compute-integral-term compute-derivative-term
;;;            |                 \          |              / 
;;;            |                 compute-total-correction
;;;            |                            |
;;;          error                        correction
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;; Ideally this version of AWDRAT would model state-variables like the code in natsoft does
;;; but we don't so I'll hack around that with tracers etc, keeping the state in the observation
;;; model
;;; In order to do that we have events for the computational steps that update internal state:
;;; Accumulate Error & Update State.  This lets us watch state updates and use the state
;;; in the next observational round.

(register-event controller-step respond-to-sensor-value)
;; (register-event estimate-error (flet respond-to-sensor-value compute-error))
;;; These two calculate the I & D values, given the error and the internal state
;; (register-event compute-derivative (flet respond-to-sensor-value compute-new-derivative))
;; (register-event compute-integral (flet respond-to-sensor-value compute-new-integral))
;;; These three are the steps that weight P,I and D values
(register-event compute-proportional-term (flet respond-to-sensor-value compute-new-proportional-term))
(register-event compute-derivative-term (flet respond-to-sensor-value compute-new-derivative-term))
(register-event compute-integral-term (flet respond-to-sensor-value compute-new-integral-term))
;;; The conceptual step of computing the correction begins with compute-correction and ends with clip-correction
;;; There's an internal data flow between these two, but we'll ignore that level of detail
;; (register-event compute-correction (flet respond-to-sensor-value compute-total-correction))
;; (register-event clip-correction (flet respond-to-sensor-value clip))

;;; The two steps for updating internal state.  Need tracers for these
;; (register-event accumulate-error (flet respond-to-sensor-value accumulate-error))
;; (register-event update-state (flet respond-to-sensor-value update-state))

;;; Note: At the moment because the top level has no components
;;; none of this gets invoked.  Just using tracers on the three computation steps.
(define-component-type controller-step
    :primitive nil
    :entry-events (controller-step)
    :exit-events (controller-step)
    :allowable-events (update-state accumulate-error)
    :inputs (controller observation dt)
    :outputs (command error)
    :behavior-modes (normal)
    :components ((estimate-error :type estimate-error :models (normal))
		 (compute-derivative :type compute-derivative :models (normal))
		 (compute-integral :type compute-integral :models (normal))
		 (compute-proportional-term :type compute-proportional-term :models (normal changed-paramter))
		 (compute-derivative-term :type compute-derivative-term :models (normal changed-parameter))
		 (compute-integral-term :type compute-integral-term :models (normal changed-paramter))
		 (compute-correction :type compute-correction :models (normal distorted)))
    :dataflows ((observation controller-step observation estimate-error)
		(the-error estimate-error the-error compute-derivative)
		(the-error estimate-error the-error compute-integral)
		(the-error estimate-error proportional compute-proportional-term)
		(derivative compute-derivative derivative compute-derivative-term)
		(integral compute-integral integral compute-integral-term)
		(weighted-proportional compute-proportional-term proportional compute-correction)
		(weighted-integral compute-integral-term integral compute-correction)
		(weighted-derivative compute-derivative-term derivative compute-correction)
		(total-correction compute-correction command controller-step)
		(the-error estimate-error error controller-step)
		)
    )

(define-component-type estimate-error
    :primitive t
    :entry-events (estimate-error)
    :exit-events (estimate-error)
    :inputs (observation)
    :outputs (the-error)
    :behavior-modes (normal)
    )

(defbehavior-model (estimate-error normal)
    :inputs (observation)
    :outputs (the-error)
    :prerequisites ([data-type-of ?observation number])
    :post-conditions ([data-type-of ?observation number])
    )

(define-component-type compute-integral
    :primitive t
    :entry-events (compute-integral)
    :exit-events (compute-integral)
    :inputs (the-error)
    :outputs (integral)
    :behavior-modes (normal)
    )

(defbehavior-model (compute-integral normal)
    :inputs (the-error)
    :outputs (integral)
    :prerequisites ([data-type-of ?the-error number])
    :post-conditions ([data-type-of ?integral number])
    )

(define-component-type compute-derivative
    :primitive t
    :entry-events (compute-derivative)
    :exit-events (compute-derivative)
    :inputs (the-error)
    :outputs (derivative)
    :behavior-modes (normal)
    )

(defbehavior-model (compute-derivative normal)
    :inputs (the-error)
    :outputs (derivative)
    :prerequisites ([data-type-of ?the-errror number])
    :post-conditions ([data-type-of ?derivative number])
    )

(define-component-type compute-proportional-term
    :primitive t
    :entry-events (compute-proportional-term)
    :exit-events (compute-proportional-term)
    :inputs (proportional)
    :outputs (weighted-proportional)
    :behavior-modes (normal)
    )

(defbehavior-model (compute-proportional-term normal)
    :inputs (proportional)
    :outputs (weighted-proportional)
    :prerequisites ([data-type-of ?proportional number])
    :post-conditions ([data-type-of ?weighted-proportional number]))

(define-component-type compute-intergral-term
    :primitive t
    :entry-events (compute-integral-term)
    :exit-events (compute-integral-term)
    :inputs (integral)
    :outputs (weighted-integral)
    :behavior-modes (normal)
    )

(defbehavior-model (compute-integral-term normal)
  :inputs (integral)
  :outputs (weighted-integral)
  :prerequisites ([data-type-of ?integral number])
  :post-conditions ([data-type-of ?weighted-integral number]))

(define-component-type compute-derivative-term
    :primitive t
    :entry-events (compute-derivative-term)
    :exit-events (compute-derivative-term)
    :inputs (derivate)
    :outputs (weighted-derivative)
    :behavior-modes (normal)
    )

(defbehavior-model (compute-derivative-term normal)
  :inputs (derivative)
  :outputs (weighted-derivative)
  :prerequisites ([data-type-of ?derivative number])
  :post-conditions ([data-type-of ?weighted-derivative number]))

(define-component-type compute-correction
    :Primitive t
    :entry-events (compute-correction)
    :exit-events (clip-correction)
    :inputs (proportional integral derivative)
    :outputs (total-correction)
    :behavior-modes (normal)
    )

(defbehavior-model (compute-correction normal)
    :inputs (proportional integral derivative)
    :outputs (total-correction)
    :prerequisites ([data-type-of ?proportional number]
		    [data-type-of ?integral number]
		    [data-type-of ?derivative number])
    :post-conditions ([data-type-of ?total-correction number])
    )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun get-plant-parameters (plant)
  (let* ((parameter-names (parameters-of-interest plant)))
    (loop for (name) in parameter-names
	for value = (funcall name plant)
	for keyword = (intern (string name) :keyword)
	collect keyword
	collect value)))

(defun copy-a-plant (plant)
  (apply #'make-instance (type-of plant) (get-plant-parameters plant)))

(defclass plant-monitor ()
  ((reference-plant :initform nil :initarg :reference-plant :accessor reference-plant)
   (reference-controller :Initform nil :Initarg :reference-controller :accessor reference-controller)
   (time-step :initform nil :initarg :time-step :accessor time-step)
   (initialized? :initform nil :accessor initialized?)
   (simulated-time :initform 0 :accessor simulated-time)
   (divergence-threshold :initform .7 :accessor divergence-threshold)
   (predicted-sensor-sequence :initform (make-array 20 :fill-pointer 0 :adjustable t) :accessor predicted-sensor-sequence)
   (error-detected :initform nil :accessor error-detected)
   (miscalculation :initform nil :accessor miscalculation)
   (time-step-number :initform 0 :accessor time-step-number)
   (residual-histogram :accessor residual-histogram :initform (make-array 43 :element-type 'integer :initial-element 0))
   )
  )

(defclass monitor-controller (pid-controller)
  ((kp-term :accessor kp-term)
   (ki-term :accessor ki-term)
   (kd-term :accessor kd-term)
   (correction :accessor correction)))

(defmethod copy-pid ((controller pid-controller))
  (make-instance 'monitor-controller
    :kp (kp controller)
    :kd (kd controller)
    :ki (ki controller)
    :setpoint (setpoint controller)
    :upper-limit (upper-limit controller)
    :lower-limit (lower-limit controller)
    ))

(defmethod accumulate-residual-statistic ((monitor plant-monitor) residual)
  (let* ((histogram (residual-histogram monitor))
	 (plant (reference-plant monitor))
	 (sigma (sensor-noise-sigma plant))
	 (six-sigma (* sigma 6))
	 (twelve-sigma (* six-sigma 2))
	 (bucket-size (/ twelve-sigma (- (length histogram) 2)))
	 (offset (round (length histogram) 2))
	 (raw-index (round residual bucket-size))
	 (index (+ offset raw-index -1))
	 )
    (cond ((< residual (- six-sigma))
	   (setq index 0))
	  ((> residual six-sigma)
	   (setq index (1- (length histogram)))))
    (incf (aref histogram index))
   ))

(defmethod controller-step ((controller monitor-controller) observation dt)
  (with-slots (signal-error integral kp ki kd setpoint upper-limit lower-limit 
	       proportional integral derivative correction kp-term ki-term kd-term) controller
    (flet ((clip (value)
	     (when lower-limit (setq value (max lower-limit value)))
	     (when upper-limit (setq value (min upper-limit value)))
	     value))
      (let* ((new-signal-error (- observation setpoint))
	     (new-proportional new-signal-error)
	     (new-derivative (derivative-slice signal-error dt))
	     (new-integral (integral-slice signal-error dt)))
	(incf integral new-integral)
	(setf derivative new-derivative
	      proportional new-proportional
	      signal-error new-signal-error
	      kp-term (* kp proportional)
	      ki-term (* ki integral)
	      kd-term (* kd derivative)
	      correction  (clip (- (+ kp-term ki-term kd-term))))
	))))


;;; Notes: What I'm going to do is have the Monitor include both a PID controller and a plant model.
;;; On each step, it will present the sensor value to the controller and the command of the real controller to the plant model.
;;; Since presenting the sensor value to the monitor's controller will precompute all the values
;;; All these tracers need to do is to check against those.
 
(deftracer (plant-startup entry) (frame)
  (let* ((my-plant-copy (copy-a-plant (plant frame)))
	 (my-controller-copy (copy-pid (controller frame)))
	 (monitor (make-instance 'plant-monitor :reference-plant my-plant-copy :reference-controller my-controller-copy)))
    (setf (awdrat-information frame) monitor)
    ;; (format *error-output* "~%Starting up with ~{~a ~a ~^, ~}" (get-plant-parameters my-plant-copy))
    (values)
    ))

(deftracer (controller-step entry) (controller observation dt)
  (declare (ignore controller))
  (let ((monitor-information (awdrat-information clim:*application-frame*)))
    (with-slots (initialized? time-step plant-output-name reference-plant reference-controller simulated-time 
		 time-step-number divergence-threshold predicted-sensor-sequence)
	monitor-information
      ;; Let our reference controller compute ahead what should happen
      (controller-step reference-controller observation dt)
      (when (not initialized?)
	(setq time-step dt initialized? t)
	(funcall (fdefinition `(setf ,(plant-state-name reference-plant))) observation reference-plant))
      ;; (format *error-output* "~%Noticing plant output ~a at time ~a" observation simulated-time)
      ;; Note: Since this is on entry to the controller, the predicted value is in response to the last command
      ;; as is the sensor value, and so these two things should be in correspondence
      ;; Now computer what our reference plant would do
      (let* ((predicted-output (funcall (plant-state-name reference-plant) reference-plant))
	     (residual (- predicted-output observation)))	
	(vector-push-extend predicted-output predicted-sensor-sequence)
	;; (format *error-output* "~%Predicted output is ~a at time ~a" predicted-output simulated-time)
	(unless (< (abs residual) divergence-threshold)
	  (setf (error-detected monitor-information) 
	    (list time-step-number simulated-time predicted-output observation))
	  ;; (format *error-output* "~%Bailing out in controller step entry")
	  (throw 'stop-the-simulation (values))
	  )
	(incf time-step-number)
	))))

(deftracer (controller-step exit) (command error)
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information))
	 (expected-command (correction reference-controller)))
    (accumulate-residual-statistic monitor-information error)
    (unless (= command expected-command)
      ;; if there's already a miscalculation don't overwrite it
      (setf (miscalculation monitor-information) (list 'command command expected-command))
      ;; (format *error-output* "~%Bailing out in controller step exit")
      (throw 'stop-the-simulation (values)))
    (with-slots (simulated-time time-step reference-plant) monitor-information
      (incf simulated-time time-step)
      ;; (format *error-output* "~%Applied command to simulated plant")
      (respond-to-command reference-plant command time-step))))

(deftracer (compute-proportional-term entry) (the-error)
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information))
	 (expected-error (signal-error reference-controller)))
    ;; (format *error-output* "~%Controller kp Input ~a" the-error )
    (unless (= the-error expected-error)
      (setf (miscalculation monitor-information) (list 'kp-input the-error expected-error))
      ;; (format *error-output* "~%Bailing out in proportional term entry")
      (throw 'stop-the-simulation (values))))
    )

(deftracer (compute-proportional-term exit) (what-he-did)
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information))
	 ;; (controller-kp (kp reference-controller))
	 (expected-kp (kp-term reference-controller)))
    ;; (format *error-output* "~%Controller kp ~a Expected Output ~a Output ~a" controller-kp expected-kp what-he-did)
    (unless (= what-he-did expected-kp)
      (setf (miscalculation monitor-information) (list 'kp what-he-did expected-kp))
      ;; (format *error-output* "~%Bailing out in proportional term exit ~a ~a" what-he-did expected-kp)
      (throw 'stop-the-simulation (values)))))

(deftracer (compute-integral-term entry) (the-integral)
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information))
	 (expected-integral (integral reference-controller)))
    (unless (= the-integral expected-integral)
      (setf (miscalculation monitor-information) (list 'ki-input the-integral expected-integral))
      ;; (format *error-output* "~%Bailing out in integral term entry ~a ~a" the-integral expected-integral)
      (throw 'stop-the-simulation (values))))
    )

(deftracer (compute-integral-term exit) (what-he-did)
  (let* ((monitor-information (awdrat-information clim:*application-frame*)) 
	 (reference-controller (reference-controller monitor-information))
	 (expected-ki (ki-term reference-controller)))
    (unless (= what-he-did expected-ki )
      (setf (miscalculation monitor-information) (list 'ki what-he-did expected-ki))
      ;; (format *error-output* "~%Bailing out in integral term exit ~a ~a" what-he-did expected-ki)
      (throw 'stop-the-simulation (values)))))

(deftracer (compute-derivative-term entry) (the-derivative)
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information))
	 (expected-derivative (derivative reference-controller)))
    (unless (= the-derivative expected-derivative)
      (setf (miscalculation monitor-information) (list 'kd-input the-derivative expected-derivative))
      ;; (format *error-output* "~%Bailing out in derivative term entry")
      (throw 'stop-the-simulation (values)))))

(deftracer (compute-derivative-term exit) (what-he-did)
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information))
	 (expected-kd (kd-term reference-controller)))
    (unless (= what-he-did expected-kd)
      (setf (miscalculation monitor-information) (list 'kd what-he-did expected-kd))
      ;; (format *error-output* "~%Bailing out in derivative term exit")
      (throw 'stop-the-simulation (values)))))
	
  