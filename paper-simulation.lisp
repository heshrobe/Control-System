;;; -*- Mode: Common-lisp; Package: Awdrat -*-

(in-package :awdrat)

;;; system parameters

(defparameter *system-matrix* #2A((1.0d0 0.0d0) (1.0d0 1.0d0)))

(defparameter *kalman-matrix* #2A((0.5939d0 0.0793d0) (0.0793d0 0.6944d0)))

;;; This is the B, i.e. the thing that multiplies the control input
(defparameter *control-vector* #(1.0d0 0.5d0))

;;; This is the control law vector that computes the next control
(defparameter *control-law-vector* #(-1.0285d0 -0.4345d0))

;;; SHould be (diag 0 1)
(defparameter *sensor-selection-matrix* #2A((0.0d0 0.0d0) (0.0d0 1.0d0)))

(defparameter *set-point* #(0.0d0 100.0d0))

(defparameter *initial-state* #(0.0d0 100.0d0))

(defparameter *noise-sigma* 0.1d0)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; In this poor man's matrix code, the first index is row number
;;; (usually i) and the second is column number (usually j)
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun mmult (matrix1 matrix2)
  ;; assuming middle dim is right
  (let* ((dim1 (array-dimension matrix1 0))
	 (dim3 (array-dimension matrix1 1))
	 (dim2 (array-dimension matrix2 1))
	 (new-matrix (make-array (list dim1 dim2))))
    (loop for i below dim1
	do (loop for j below dim2
	       do (setf (aref new-matrix i j)
		    (loop for k below dim3
			sum (* (aref matrix1 i k)
			       (aref matrix2 k j))))))
    new-matrix))

(defun madd (matrix1 matrix2)
  (let* ((dim1 (array-dimension matrix1 0))
	 (dim2 (array-dimension matrix1 1))
	 (new-matrix (make-array (list dim1 dim2))))
    (loop for i below dim1
	do (loop for j below dim2
	       do (setf (aref new-matrix i j) (+ (aref matrix1 i j) (aref matrix2 i j)))))
    new-matrix
    ))

(defun mminus (matrix1 matrix2)
  (let* ((dim1 (array-dimension matrix1 0))
	 (dim2 (array-dimension matrix1 1))
	 (new-matrix (make-array (list dim1 dim2))))
    (loop for i below dim1
	do (loop for j below dim2
	       do (setf (aref new-matrix i j) (- (aref matrix1 i j) (aref matrix2 i j)))))
    new-matrix
    ))

(defun diag (list)
  (let* ((dim (length list))
	 (matrix (make-array (list dim dim))))
    (loop for i below dim do (setf (aref matrix i i) (pop list)))
    matrix))

(defun M* (matrix vector)
  ;; assuming square matrix
  (let* ((length (length vector))
	 (new-vector (make-array length)))
    (loop for i below length
	do (setf (aref new-vector i)
	     (loop for j below length
		 sum (* (aref matrix i j) (aref vector j)))))
    new-vector))

(defun v+ (&rest vectors)
  (let* ((length (length (first vectors)))
	 (new-vector (make-array length)))
    (loop for i below length
	do (setf (aref new-vector i)
	     (loop for v in vectors
		 sum (aref v i))))
    new-vector))

(defun v- (v1 v2)
  (Unless (= (length v1) (length v2)) (error "Inconsistent Vectors"))
  (let* ((length (length v1))
	 (new-vector (make-array length)))
    (loop for i below length
	do (setf (aref new-vector i)
	     (- (aref v1 i) (aref v2 i))))
    new-vector))

;;; scalar multiplication
(defun s* (vector scalar)
  (let* ((length (length vector))
	 (new-vector (make-array length)))
    (loop for i below length
	do (setf (aref new-vector i)
	     (* scalar (aref vector i))))
    new-vector))

(defun dot* (v1 v2)
  (loop for i below (length v1)
      sum (* (aref v1 i) (aref v2 i))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Simulation
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; The predictor of the Kalman-filter
(defun next-state-estimate (current-state-estimate current-control)
  (v+ current-state-estimate
      (s* *control-vector* current-control)))

;;; The corretor of the Kalman-filter
(defun corrected-state-estimate (current-state-estimate residual)
  (v+ current-state-estimate
      (m* *kalman-matrix* residual)))

;;; What the system actually does including noise
(defun next-system-state (current-state control &optional (add-noise t))
  (let ((core (v+ (m* *system-matrix* current-state)
		  (s* *control-vector* control)))
	(noise (when add-noise (next-system-noise))))
    (list (if add-noise (v+ core noise) core) noise)))

(defun next-system-noise (&optional (velocity-sigma *noise-sigma*) (position-signma *noise-sigma*))
  (let ((velocity-componenent (randist:random-normal 0 velocity-sigma))
	(position-component (randist:random-normal 0 position-signma)))
    (make-array 2 :initial-contents (list velocity-componenent (+ position-component (* .5 velocity-componenent))))))

;;; What we sense including noise
(defun system-output (current-state)
  (let ((sensor-noise (next-sensor-noise)))
    (list (v+ current-state sensor-noise)
	  sensor-noise)))

(defun next-sensor-noise (&optional (velocity-sigma *noise-sigma*) (position-signma *noise-sigma*))
  (let ((velocity-componenent (randist:random-normal 0 velocity-sigma))
	(position-component (randist:random-normal 0 position-signma)))
    (make-array 2 :initial-contents (list velocity-componenent position-component))))

(defun compute-attack-data (nsteps)
  (let ((vector (make-array nsteps)))
    (loop for time below nsteps
	do (setf (aref vector time)
	     (cond
	      ((= time 0) #(0.0d0 -1.0d0))
	      ((= time 1) #(0.0d0 -0.367d0))
	      (t (let ((prior (aref vector (- time 2))))
		   (v- prior #(0.0d0 -0.485d0)))))))
    vector))

(defvar *verbose-output* nil)

(defun v-format (stream format-string &rest args)
  (when *verbose-output*
    (apply #'format stream format-string args)))

(defparameter *all-signals* '(position residual output error corrupted-output attack-data))

(defun evolve-system (nsteps &key with-attack? verbose? (x-scale 5) (y-scale 5) (noise-sigma .5) (plot? t) (print? t) (signals-to-plot *all-signals*))
  (let* ((*verbose-output* verbose?)
	 (*noise-sigma* noise-sigma)
	 (positions `(position ,(make-array nsteps :fill-pointer 0)))
	 (residuals `(residual ,(make-array nsteps :fill-pointer 0)))
	 (output `(output ,(make-array nsteps :fill-pointer 0)))
	 (errors `(error ,(make-array nsteps :fill-pointer 0)))
	 (corrupted-data `(corrupted-output ,(make-array nsteps :fill-pointer 0)))
	 (attack-data `(attack-data ,(make-array nsteps :fill-pointer 0)))
	 (data-vector (list positions residuals output errors corrupted-data attack-data)))
    (loop with attack-vector = (when with-attack? (compute-attack-data nsteps))
	for time below nsteps
	for current-control = 0 then next-control
	for (current-state system-noise) = (list *initial-state* nil) then  (next-system-state current-state current-control :add-noise)
	for current-state-estimate = *initial-state* then (next-state-estimate current-state-estimate current-control)
	for predicted-output = current-state-estimate
	for (actual-output sensor-noise) = (system-output current-state)
	for injected-attack = (when with-attack? (M* *sensor-selection-matrix* (aref attack-vector time)))
	for corrupted-output = (when with-attack? (v+ actual-output injected-attack))
	for residual = (v- (if with-attack? corrupted-output actual-output) predicted-output)
	for corrected-state-estimate = (corrected-state-estimate current-state-estimate residual)
	for error = (v- corrected-state-estimate *set-point*)
	for next-control = (dot* *control-law-vector* error)
	do  (vector-push (aref residual 1) (second residuals))
	    (vector-push (aref current-state 1) (second positions))
	    (vector-push (aref actual-output 1) (second output))
	    (vector-push (aref error 1) (second errors))
	when with-attack?
	do (vector-push (aref corrupted-output 1) (second corrupted-data))
	   (vector-push (aref injected-attack 1) (second attack-data))
	do (v-format t "~2%Current State ~a ~%Noise ~a" current-state system-noise)
	   (v-format t "~%Current State Estimate ~a" current-state-estimate)
	   (v-format t "~%Output ~a ~%Noise ~a" actual-output sensor-noise)
	   (when with-attack? (v-format t "~%Attack data ~a" injected-attack)
		 (v-format t "~%Corrupted output ~a" corrupted-output))
	   (v-format t "~%Residual ~a" residual)
	   (v-format t "~%Corrected state estimate ~a" corrected-state-estimate)
	   (v-format t "~%Control ~a" next-control)
	finally (when plot?
		  (let ((data (loop for name in signals-to-plot collect (second (assoc name data-vector)))))
		    (format t "~%Displaying ~a" signals-to-plot)
		    (Plot data						      
			  :colors *color-list*
			  :x-scale x-scale
			  :y-scale y-scale
			  :x-value-increment 1
			  :x-grid-increment 20
			  :parameter-names signals-to-plot
			  )))
		(when print?
		  (terpri)
		  (let* ((residuals (second residuals))
			 (average (average residuals)))
		    (format t "~%Average residual ~a, sigma ~a, skewness ~a" 
			    average
			    (sigma residuals average)
			    (skewness residuals average)))
		  (let* ((errors (second errors))
			 (average (average errors)))
		    (format t "~%Average error ~a, sigma ~a, skewness ~a" 
			    average
			    (sigma errors average)
			    (skewness errors average))
		    
		    (format t "~%Average offset from setpoint ~a" (average errors))))
		(return (average (second errors)))
		)))

(defun average (stuff)
  (/ (reduce #'+ stuff) (length stuff)))

(defun sigma (stuff &Optional average)
  (let ((average (or average (average stuff))))
    (/ (sqrt (loop for element across stuff for difference = (- element average) sum (* difference difference)))
       (length stuff)
    )))

(defun skewness (stuff &Optional average)
  (let* ((average (or average (average stuff)))
	 (N (length stuff))
	 (sum-of-diffs-squared (loop for element across stuff for difference = (- element average) sum (* difference difference)))
	 (sum-of-diffs-cubed (loop for element across stuff for difference = (- element average) sum (* difference difference difference))))
    (/ (/ sum-of-diffs-cubed N)
       (expt (/ sum-of-diffs-squared N) 1.5))))
    
	    
    
    
    
    

(defun average-r-square (v-list n-steps)
  (loop for v in v-list
      sum (dot* v v) into sum
      finally (return (/ sum n-steps))))

;;; (A - KCA) since C is identity
(defparameter *m1* (mminus *system-matrix* (mmult *kalman-matrix* *system-matrix*)))

;;; KDelta
(defparameter *m2* (mmult *kalman-matrix* *sensor-selection-matrix*))

(defparameter *y0* #(0 -1))
(defparameter *y1* #(0 -0.367))
(defparameter *delta-e-minus-one* #(0 0))

(defun evolve (old-delta-e y) 
  (v-
   (M* *m1* old-delta-e)
   (M* *m2* y)))
       