;;; -*- Mode: Commonlisp; package: Awdrat; Readtable: Joshua -*-

(in-package :awdrat)

;;; This is now in a separate file called trace-format that is
;;; loaded before everything else
;;; (defparameter *do-tracing* nil)

;;; (defmacro trace-format (control-string &rest args)
;;;   `(when *do-tracing*
;;;      (format *error-output* ,control-string ,@args)))

;;; The plant events
;;; We just notice that the plant starts up
;;; The rest is done by tracer-sxx
;;; Note: This could just as correctly wrap do-a-simulation
(register-event plant-startup simulate)

(define-component-type plant-startup
    :top-level t
    :entry-events (plant-startup)
    :exit-events (plant-startup)
    :allowable-events (controller-step plant-step compute-error compute-proportional-term compute-integral-term compute-derivative-term)
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

;;; The two events we folow are when the plant gets a command (respond-to-command)
;;; and when a sensor value is observed (respond-to-sensor-value)
(register-event plant-step respond-to-command)
(register-event controller-step respond-to-sensor-value)
;;; Respond-to-sensor-value estimates the state then calculates the appropriate
;;; command to issue.  It's substeps in addition to esimating the state include computing the P,I and D terms
;;; and then summing them.
;;; The way this monitor works is that when the respond-to-sensor-value event is noticed
;;; it precomputes what should happen, stores those results and then returns.
;;; The sub-event tracers below, then just check that what the "real" system did corresponds to
;;; these saved values.

;;; This step computes the error, i.e. the difference between the set-point and the estimated-state
(register-event compute-error respond-to-sensor-value-compute-error)

;;; This step estimates the state of the plant given the sensor value, past state and the last command issued
;;; I.e. it runs a Kalman filter
;; (register-event estimate-state (flet respond-to-sensor-value estimate-state))
;;; These three are the steps that weight P,I and D values
(register-event compute-proportional-term respond-to-sensor-value-compute-new-proportional-term)
(register-event compute-derivative-term respond-to-sensor-value-compute-new-derivative-term)
(register-event compute-integral-term respond-to-sensor-value-compute-new-integral-term)
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
;;;
;;; Support for Tracers
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun copy-a-plant (plant)
  (let ((plant-copy (apply #'make-instance (type-of plant) (get-plant-parameters plant))))
    (setf (simulation-frame plant-copy) (simulation-frame plant))
    (reset plant-copy)
    plant-copy))

(defun get-plant-parameters (plant)
  (let* ((parameter-names (parameters-of-interest plant)))
    (loop for (name) in parameter-names
	for value = (funcall name plant)
	for keyword = (intern (string name) :keyword)
	collect keyword
	collect value)))

(defclass plant-monitor ()
  ((reference-plant :initform nil :initarg :reference-plant :accessor reference-plant)
   (reference-controller :Initform nil :Initarg :reference-controller :accessor reference-controller)
   (initialized? :initform nil :accessor initialized?)
   ;; The size of dt
   (time-step :initform nil :initarg :time-step :accessor time-step)
   ;; This increments by one on each step
   (time-step-number :initform 0 :accessor time-step-number)
   ;; this increments by dt each step
   (simulated-time :initform 0 :accessor simulated-time)
   (current-observation :accessor current-observation)
   (divergence-threshold :initform .7 :accessor divergence-threshold)
   (predicted-sensor-sequence :initform (make-array 20 :fill-pointer 0 :adjustable t) :accessor predicted-sensor-sequence)
   (error-detected :initform nil :accessor error-detected)
   (miscalculation :initform nil :accessor miscalculation)
   (residual-histogram :accessor residual-histogram :initform (make-array 43 :element-type 'integer :initial-element 0))
   ))

(defclass monitor-controller (pid-controller)
  (;; these three are here to hold the results of each step's
   ;; computation made in the entry tracer so that it can be checked
   ;; against the returned-value in the exit-tracer
   (proportional-term :accessor proportional-term)
   (integral-term :accessor integral-term)
   (derivative-term :accessor derivative-term)
   (correction :accessor correction :initform 0)
   (kalman-filter :accessor kalman-filter :initarg :kalman-filter)
   ;; This is to allow us to run a separate Kalman-filter with different
   ;; parameters which means that we'll be seeing different error terms
   ;; and so our integral will be different from that of the controller
   ;; being monitored.
   (monitor-integral :accessor monitor-integral :initform 0)
   ;; calculating the integral and derivative slices requires us to have both the current error
   ;; and the last error
   (current-signal-error :accessor current-signal-error :Initform 0)
   (last-signal-error :accessor last-signal-error :Initform 0)
   ))

(defmethod copy-pid ((controller pid-controller) kalman-filter)
  (make-instance 'monitor-controller
    :kp (kp controller)
    :kd (kd controller)
    :ki (ki controller)
    :setpoint (setpoint controller)
    :upper-limit (upper-limit controller)
    :lower-limit (lower-limit controller)
    :kalman-filter kalman-filter
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

;;; Notes: What I'm going to do is have the Monitor include both a PID controller and a plant model.
;;; On each step, it will present the sensor value to the controller and the command of the real controller to the plant model.
;;; Since presenting the sensor value to the monitor's controller will precompute all the values
;;; All these tracers need to do is to check against those.

(deftracer (plant-startup entry) (frame)
  (let* ((my-plant-copy (copy-a-plant (plant frame)))
	 ;; notice that copy-a-plant makes a copy and does reset on it, so my kalman-filter starts
	 ;; where the real guy's filter starts.
	 (my-state-estimator (funcall (kalman-filter-constructor my-plant-copy) my-plant-copy (delta-time frame)))
	 (my-controller-copy (copy-pid (controller frame) my-state-estimator))
	 (monitor (make-instance 'plant-monitor :reference-plant my-plant-copy :reference-controller my-controller-copy)))
    (setf (awdrat-information frame) monitor)
    (trace-format "~%Starting up with ~{~a ~a ~^, ~}" (get-plant-parameters my-plant-copy))
    (values)
    ))

;;; The monitor's respond-to-command
(deftracer (plant-step entry) (plant control dt)
  (declare (ignore plant))
  (let ((monitor-information (awdrat-information clim:*application-frame*)))
    (with-slots (simulated-time time-step reference-plant) monitor-information
      (trace-format "~%Applied command ~a to simulated plant" control)
      (with-slots (height pump-rate pump-rate-scale drain-rate volume area) reference-plant
	(trace-format "~%Reference Plant responding to command ~a, current height ~a pump rate ~a" control height pump-rate)
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
	  (trace-format "~%Reference plant new height ~d" height)
	  (values)
	  )))))

;;; In the alist is the output of the plant
;;; We should check that this is similar to the output
;;; of the reference plant but then squirrel away the output of the real-plant
(deftracer (plant-step exit) (alist)
  (declare (ignore alist))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; The steps of the controller
;;;
;;; The approach in each of these is to compute what the "real" system would do in
;;; the entry tracer and then squirrel away the result in the monitor
;;; so that it be checked in the exit tracer against what the "real" system returns
;;;
;;; Each of the steps corresponds to the compute-xxx-term method in the controller
;;; not to the compute-new-xxx (e.g. compute new-integral)
;;; These get the result of compute-new-xxx as input
;;; In these tracers we fold that into the tracer
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; This is invoked when sensor value is reported (i.e. it traces respond-to-sensor-value)
(deftracer (controller-step entry) (controller previous-command observation dt)
  (declare (ignore controller previous-command))
  (trace-format "~%Controller got observation ~a" observation)
  (let ((monitor-information (awdrat-information clim:*application-frame*)))
    (with-slots (initialized? time-step simulated-time time-step-number current-observation) monitor-information
      (when (not initialized?)
	(setq time-step dt
	      time-step-number 0
	      simulated-time 0
	      initialized? t))
      (setq current-observation observation)
      (incf time-step-number)
      (incf simulated-time dt)
      (trace-format "~%Monitor noticing sensor value ~a at time ~a" observation simulated-time)
      )))

;;; By the time that this gets invoked, the sub-steps have done their
;;; thing and stored away the integral, derivative and proportional terms.
;;; we just need to add them up and check that it's the same as the command
(deftracer (controller-step exit) (command error residual)
  (declare (ignore error residual))
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information)))
    (with-slots (lower-limit upper-limit proportional-term integral-term derivative-term) reference-controller
      (flet ((clip (value)
	       (when lower-limit (setq value (max lower-limit value)))
	       (when upper-limit (setq value (min upper-limit value)))
	       value))
	(trace-format "~%Proportional ~a Derivative ~a Integral ~a:" proportional-term integral-term derivative-term)
	(let ((expected-command (clip (- (+ proportional-term integral-term derivative-term)))))
	  (trace-format "~%In controller-step exit got ~a expected ~a" command expected-command)
	  (unless (= command expected-command)
	    ;; if there's already a miscalculation don't overwrite it
	    (setf (miscalculation monitor-information) (list 'command command expected-command))
	    (trace-format "~%Bailing out in controller step exit")
	    (throw 'stop-the-simulation (values)))
	  )))))

;;; Notice here that wer're getting passed the "real" system's state-estimate
;;; and we've stored away the observation as well.
;;; So we can compute the same error as "real" system
;;; and store that away to check against in the exit tracer
(deftracer (compute-error entry) (controller state-estimate)
  (declare (ignore controller))
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (observation (current-observation monitor-information))
	 (reference-controller (reference-controller monitor-information))
	 (set-point (setpoint reference-controller))
	 (residual (- state-estimate observation)))
    (accumulate-residual-statistic monitor-information residual)
    (with-slots (time-step-number simulated-time predicted-sensor-sequence divergence-threshold) monitor-information
      ;; (vector-push-extend estimated-state predicted-sensor-sequence)
      (unless (< (abs residual) divergence-threshold)
	(setf (error-detected monitor-information)
	  (list time-step-number simulated-time state-estimate observation))
	(trace-format "~%Bailing out in controller step entry, residual is ~a" residual)
	(throw 'stop-the-simulation (values)))
      ;; having computed the error, save the current-error from the last step
      ;; as the last-error and save the computed error to check against in the exit tracer
      (let ((current-error (- state-estimate set-point)))
	(trace-format "~%Monitor calculated error ~a" current-error)
	(setf (last-signal-error reference-controller)  (current-signal-error reference-controller)
	      (current-signal-error reference-controller) current-error))
      )))

(deftracer (compute-error exit) (new-signal-error)
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information))
	 (current-signal-error (current-signal-error reference-controller)))
    ;; This is the signal error that the real controller
    ;; returned.  Make sure that it matches what we expect.
    (unless (= new-signal-error current-signal-error)
      (setf (error-detected monitor-information)
	(list current-signal-error new-signal-error))
      (trace-format "~%Bailing out in controller step exit expeced error is ~a got ~A" current-signal-error new-signal-error)
      (throw 'stop-the-simulation (values))
      )
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Proportional term
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(deftracer (compute-proportional-term entry) (controller proportional)
  (declare (ignore controller))
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information))
	 (kp (kp reference-controller))
	 (expected-error (current-signal-error reference-controller))
	 (expected-proportional expected-error))
    (trace-format "~%Controller kp Input ~a expected ~a" proportional expected-proportional)
    (unless (= proportional expected-proportional)
      (setf (miscalculation monitor-information) (list 'kp-input proportional expected-proportional))
      (trace-format "~%Bailing out in proportional term entry")
      (throw 'stop-the-simulation (values)))
    (setf (proportional-term reference-controller) (* kp proportional))
    )
  )

(deftracer (compute-proportional-term exit) (what-he-did)
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information))
	 (expected-proportional-term (proportional-term reference-controller)))
    (unless (= what-he-did expected-proportional-term)
      (setf (miscalculation monitor-information) (list 'kp what-he-did expected-proportional-term))
      (trace-format "~%Bailing out in proportional term exit ~a ~a" what-he-did expected-proportional-term)
      (throw 'stop-the-simulation (values)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Integral-term
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(deftracer (compute-integral-term entry) (controller the-integral)
  (declare (ignore controller))
  (trace-format "~%In monitor compute-integral-term got ~a" the-integral)
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information)))
    (with-slots ((dt time-step)) monitor-information
      (with-slots ((signal-error last-signal-error) (new-signal-error current-signal-error)  integral ki) reference-controller
	(trace-format "~%In compute integral term old-signal-error ~a current-signal-error ~a dt ~a"
		      signal-error new-signal-error dt)
	(let ((expected-integral (integral-slice signal-error dt)))
	  (trace-format "~%Huh ~a"  (* (/ (+ signal-error new-signal-error) 2) dt))
	  (trace-format "~%Integral term ~a Expected ~a" the-integral expected-integral)
	  (unless (= the-integral expected-integral)
	    (setf (miscalculation monitor-information) (list 'ki-input the-integral expected-integral))
	    (trace-format "~%Bailing out in compute-integral-term")
	    (throw 'stop-the-simulation (values)))
	  (setf (integral-term reference-controller) (* ki expected-integral))
	  (incf integral expected-integral))))))

(deftracer (compute-integral-term exit) (what-he-did)
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information))
	 (expected-integral-term (integral-term reference-controller)))
    (unless (= what-he-did expected-integral-term)
      (setf (miscalculation monitor-information) (list 'ki what-he-did expected-integral-term))
      (trace-format "~%Bailing out in integral term exit ~a ~a" what-he-did expected-integral-term)
      (throw 'stop-the-simulation (values)))
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Derivative term
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(deftracer (compute-derivative-term entry) (controller the-derivative)
  (declare (ignore controller))
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information)))
    (with-slots ((dt time-step)) monitor-information
      (with-slots ((signal-error last-signal-error) (new-signal-error current-signal-error) kd) reference-controller
	(let ((expected-derivative (derivative-slice signal-error dt)))
	  (trace-format "~%In compute-derrivative-term derivate ~a expected ~a" the-derivative expected-derivative)
	  (unless (= the-derivative expected-derivative)
	    (setf (miscalculation monitor-information) (list 'kd-input the-derivative expected-derivative))
	    (trace-format "~%Bailing out in compute-derivative-term")
	    (throw 'stop-the-simulation (values)))
	  (setf (derivative-term reference-controller) (* kd expected-derivative)))
	))))

(deftracer (compute-derivative-term exit) (what-he-did)
  (let* ((monitor-information (awdrat-information clim:*application-frame*))
	 (reference-controller (reference-controller monitor-information))
	 (expected-kd (derivative-term reference-controller)))
    (unless (= what-he-did expected-kd)
      (setf (miscalculation monitor-information) (list 'kd what-he-did expected-kd))
      (trace-format "~%Bailing out in derivative term exit got ~a expected ~a" what-he-did expected-kd)
      (throw 'stop-the-simulation (values)))))

;;; Notes
;;; The big question is what to do about the plant model
;;; We can maintain our own kalman filter with less confidence in the sensor
;;; (i.e. bigger sigma for sensor and smaller (or none) for the plant sigma)
;;; Then we can calculate our own error term (difference between our estimated state
;;; and the observation) and our own correction.  Note that a PID controller has state
;;; in it (the integral term) so we'd need to maintain that separate from our model of the
;;; real system's PID controller.
;;; We can then apply this correction to the monitor's plant model.
;;;
;;; Then we can make the following sanity check(s)
;;; 1) Is our predicted state the same as the real system's predicted state
;;; 2) Is our estimated state the same as the real system's estimated state
;;; 3) Is our residual the same as the real system's
;;; All these need to be made within a delta that's related to the sigma's of the
;;; real system.  The real system has both sensor noise noise and process noise
;;; even when not being spoofed.  We share the sensor noise (there's only the real sensor
;;; after all) but our model may not have process noise.  The process noise is supposed to
;;; be zero-mean guassian so it shouldn't add up over time (and the controller corrects for it)
;;; so probably the real system and our model of the plant should stay within a sigma or two
;;; of each other.

(defun check-residuals ()
  (let* ((residuals (second (assoc 'residual (plot-data *control-system-frame*))))
	 (length (length residuals))
	 (mean (/ (reduce #'+ residuals) length))
	 (sigma (/ (sqrt (loop for item across residuals
			     for deviation = (- item mean)
			     for deviation-2 = (* deviation deviation)
			     sum deviation-2))
		   length)))
    (values mean sigma)))
