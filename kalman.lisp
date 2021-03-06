;;; -*- Mode: Commonlisp; package: Awdrat; Readtable: Joshua -*-

(in-package :awdrat)

#|

From: http://www.swarthmore.edu/NatSci/echeeve1/Ref/Kalman/ScalarKalman.html

Any Kalman filter operation begins with a system description consisting of gains a, b and h. 
The state is x, the input to the system is u, and the output is z. The time index is given by t.

The system model:

x(t) = ax(t-1) + bu(t) + w(t)
z(t) = hx(t) + v(t)

w is the process noise: white noise with zero mean and variance Q.
v is the observation noise: white noise with zero mean and variance R.

Note that: a, b, h, u are known.  The noise terms w and v are not.
z is observed, including its noise.
The state isn't directly observable.

So our task is to estimate the state x.

To do that (as explained below) we'll need to estimate the variance p.
   
There are two steps predictor and corrector.  
In the predictor step we calculate prior estimates of x and p.
In the corrector step we calculate posterior estimates of x and p.


(1) Predictor: 

x-prior-estimate(t) = a * x-posterior-esimate(t-1) + bu(t)
p-prior-estimate(t) = a^2 * p-posterior-estimate(t-1) + Q

Note that these two equations use previous values of the a posteriori state estimate and
covariance. Therefore the first iteration of a Kalman filter requires estimates (which are
often just guesses) of the these two variables. The exact estimate is often not important
as the values converge towards the correct value over time, a bad initial estimate just
takes more iterations to converge.

(2) Corrector: 

calculate the predicted output estimated-z(t): h * x-prior-estimate(t) 

Calculate the residual r: z(t) -  estimated-z(t)

Calculate the kalman gain k(t):      h * prior-variance-estimate(t)
                                     ---------------------------
                                   h^2 * prior-variance-estimate(t) + R

x-posterior-estimate(t) = x-prior-estimate(t) + k * (z(t) - h * x-prior-estimate(t))
p-posterior-estimate(t) = p-prior-estimate * (1 - hk(t))

|#

(defclass core-scalar-kalman-filter ()
  ((previous-state :accessor previous-state)
   (previous-variance :accessor previous-variance)
   (system-gain :accessor system-gain :initarg :system-gain :initform 1 :documentation "Usual notation A")
   (input-gain :accessor input-gain :initarg :input-gain :initform 1 :documentation "Usual notation b")
   (observation-gain :accessor observation-gain :initarg :observation-gain :documentation "Usual notation h")
   (system-variance :accessor system-variance :initarg :system-variance :documentation "Usual notation w, variance Q")
   (observation-variance :accessor observation-variance :initarg :observation-variance :documentation "Usual notation v, variance R")
   (sigma-r :accessor sigma-r :documentation "Variance of the residual for test")
   ))

(defmethod initialize-instance :after ((kf core-scalar-kalman-filter) &key initial-state initial-variance &allow-other-keys)
  (setf (previous-state kf) initial-state
	(previous-variance kf) initial-variance))

;;; This is the main method.  You call this each time an observation is made, together with 
;;; the control input to the plant.

(defmethod estimate-state ((kf core-scalar-kalman-filter) control-input observation)
  (with-slots ((h observation-gain) (R observation-variance)) kf
    (multiple-value-bind (prior-state-estimate prior-variance-estimate estimated-output) (predictor kf control-input)
      (multiple-value-bind (posterior-state-estimate posterior-variance-estimate residual) 
	  (corrector kf observation prior-state-estimate prior-variance-estimate)
	(let ((sigma-r (calculate-current-sigma-r kf prior-variance-estimate)))
	  (trace-format "~%Sigma-r ~a ~a" sigma-r (sqrt sigma-r))
	  (setf (previous-state kf) posterior-state-estimate
		(previous-variance kf) posterior-variance-estimate)
	  (trace-format "~%r^2 ~a sigma-r ~a bad ~a" (* residual residual) sigma-r (> (/ (* residual residual) sigma-r) 2))
	  (values posterior-state-estimate
		  prior-state-estimate
		  estimated-output
		  residual))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Predictor step
;;;
;;; The standard equations in the literature are implemented by the standard-scalar-kalman-filter class
;;; for the case when the system is linear.

;;;
;;; When the system is non-linear then we need to used an extended kalman filter
;;; This is subclassed to allow us to provide a method to predict the next state
;;; and the next variance.  The key difference is that you directly compute the system response
;;; and variance is computed using the partial derivative of the system response (with respect to the state)
;;; evaluated at the current state and control input
;;; Wikipedia (of course) provides a description
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod predictor ((kf core-scalar-kalman-filter) control-input)
  (with-slots ((Q system-variance)) kf
    (let ((prior-state-estimate (prior-state-estimate kf control-input)))
    (values prior-state-estimate
	    (prior-variance-estimate kf)
	    (estimated-output kf prior-state-estimate)
	    ))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Corrector Step
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

#|
(2) Corrector: 

calculate the predicted output estimated-z(t): h * x-prior-estimate(t) 

Calculate the residual r: z(t) -  estimated-z(t)

Calculate the kalman gain k(t):      h * prior-variance-estimate(t)
                                     ---------------------------
                                   h^2 * prior-variance-estimate(t) + r(t)

x-posterior-estimate(t) = x-prior-estimate(t) + k * (z(t) - h * x-prior-estimate(t))
p-posterior-estimate(t) = p-prior-estimate * (1 - hk(t))

|#

(defmethod corrector ((kf core-scalar-kalman-filter) observed-output prior-state-estimate prior-variance-estimate)
  (with-slots ((h system-gain) (R observation-variance)) kf
    (let* ((estimated-output (estimated-output kf prior-state-estimate))
	   (residual (residual kf estimated-output observed-output))
	   (kalman-gain (kalman-gain kf prior-variance-estimate)))
      (trace-format "~%Estimated output: ~a prior state estimate ~a Observed output: ~a Residual: ~a" 
		    estimated-output prior-state-estimate observed-output residual)
      (values (posterior-state-estimate kf prior-state-estimate residual kalman-gain)
	      (posterior-variance-estimate kf prior-variance-estimate kalman-gain)
	      residual))))

(defmethod estimated-output ((kf core-scalar-kalman-filter) prior-state-estimate) 
  (with-slots ((h observation-gain)) kf
      (* h prior-state-estimate)))

(defmethod residual ((kf core-scalar-kalman-filter) estimated-output observed-output) (- observed-output estimated-output))

(defmethod kalman-gain ((kf core-scalar-kalman-filter) prior-variance)
  (with-slots ((h observation-gain) (R observation-variance)) kf
    (/ (* h prior-variance)
       (+ (* h h prior-variance) R))))

(defmethod posterior-state-estimate ((kf core-scalar-kalman-filter) prior-state-estimate residual kalman-gain)
  (max 0 (+ prior-state-estimate (* kalman-gain residual))))

(defmethod posterior-variance-estimate ((kf core-scalar-kalman-filter) prior-variance-estimate kalman-gain)
  (with-slots ((h observation-gain)) kf
    (* prior-variance-estimate (1- (* h kalman-gain)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Standard Kalman Filter
;;; Provides the standard methods for calculating the next state and variance
;;; using the state gain and the state variance
;;; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass standard-scalar-kalman-filter (core-scalar-kalman-filter)
  ()
  )

;;; This method can be overridden if that state change equation is somehow more
;;; complex than the usual.  The water tank example (with the parasitic drain) is such a case.
(defmethod prior-state-estimate ((kf standard-scalar-kalman-filter) u )
  (with-slots ((a system-gain) (b input-gain) (previous-state-estimate previous-state)) kf
    (+ (* a previous-state-estimate)
       (* b u))))

(defmethod prior-variance-estimate ((kf standard-scalar-kalman-filter))
  (with-slots ((previous-variance-estimate previous-variance) (a system-gain) (Q system-variance)) kf
    (+ (* a a previous-variance-estimate) Q)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Extended Kalman Filter
;;; Really an abstract class you provide two methods to calculate new state and variance
;;; The extended filter can have both non-linear state and observation equations
;;; I'm only dealing with the state part.
;;; 
;;; Calculate F(t-1) = Partial-with-respect-to-x (state-equation)
;;;  evalulated at x(t-1) and u(t-1)
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass extended-scalar-kalman-filter (core-scalar-kalman-filter)
  ;; The state predictor method might want to stash this for the variance predictor
  ((system-derivative :accessor system-derivative))
  )

(defmethod prior-state-estimate ((kf extended-scalar-kalman-filter) u)
  (declare (ignore u))
  (error "You must provide your own method for this"))

(defmethod prior-variance-estimate ((kf extended-scalar-kalman-filter))
  (error "You must provide your own method for this")
  )