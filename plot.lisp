;;; -*- Mode: Commonlisp; package: Awdrat; Readtable: Joshua -*-

(in-package :awdrat)

(defparameter *grid-color* clim:+blue+)
(defparameter *plotting-color* clim:+black+)

(defun plot (values-arrays
	     &key
	     (colors (make-list (length values-arrays) :initial-element clim:+black+))
	     parameter-names
	     (label :None)
	     ;; this is how much of a time-interval is represented by each
	     ;; slot of the values-array
	     (x-value-increment (/ 1.0 50.0))
	     (x-scale 1)
	     (y-scale 1)
	     (left-margin 40)
	     (x-grid-increment 10)
	     (x-grid-increment-number-of-decimals 2)
	     (y-grid-increment 10)
	     (y-grid-increment-number-of-decimals 2)
	     (stream *standard-output*)
	     error-information
	     )
  (let ((min-x 0)
	(max-x (* (reduce #'max values-arrays :key #'length) x-value-increment))
	(min-y nil)
	(max-y nil))2
    (loop for array in values-arrays
	for this-min = (if (= (length array) 0) 0 (reduce #'min array))
	for this-max = (if (= (length array) 0) 0 (reduce #'max array))
	when (or (null min-y) (< this-min min-y)) do (setq min-y this-min)
	when (or (null max-y) (> this-max max-y)) do (setq max-y this-max))
    (fresh-line stream)
    (flet ((round-up (number divisor) (* divisor (ceiling number divisor)))
	   (round-down (number divisor) (* divisor (floor number divisor))))
      (let ((left (round-down min-x x-grid-increment))
	    (right (round-up max-x x-grid-increment))
	    (bottom (round-down min-y y-grid-increment))
	    (top (round-up max-y y-grid-increment))
	    (line-height (+ (clim:stream-line-height stream) 2)))
	(fresh-line stream)
	(multiple-value-bind (transform vertical-space)
	    (transform-and-y-offset-for-bbox x-scale y-scale
					     left bottom right top
					     left-margin line-height)
	  (clim:stream-increment-cursor-position stream 0 (+ (* 2 line-height) vertical-space))
	  (clim:with-first-quadrant-coordinates (stream)
	    (clim:with-drawing-options (stream :transformation transform)
	      (draw-axes left top right bottom
			 x-grid-increment x-grid-increment-number-of-decimals
			 y-grid-increment y-grid-increment-number-of-decimals
			 stream)
	      (when parameter-names
		(draw-color-key stream (/ (+ top bottom) 2) right colors parameter-names))
	      (when (and label (not (eql :none label)))
		(draw-label label stream left bottom right line-height))
	      (clim:draw-text* stream (format nil "Scale: ~5,2,,f" y-scale)
			       left top
			       :ink *grid-color*
			       :align-x :left :align-y :bottom)
	      (clim:draw-text* stream (format nil "Scale: ~5,2,,,f" x-scale)
			       right bottom
			       :align-x :left :align-y :bottom
			       :ink *grid-color*)
	      ;; draw the plot
	      (loop for array in values-arrays
		  for color in colors
		  do (draw-plot array stream x-value-increment color))
	      (when error-information
		(highlight-error stream error-information x-value-increment)))
	      ))))
	  (terpri stream)
	  (values)))


(defmethod highlight-error (stream error-information x-value-increment)
  (destructuring-bind (time-step simulated-time true-value false-value) error-information
    (declare (ignore simulated-time))
    (let ((mid-x (* time-step x-value-increment))
	  (line-height (clim:stream-line-height stream))
	  (current-transform (clim:medium-transformation stream)))
      ;; (format *error-output* "~%Sensor Spoof Detected ~a ~a ~a" simulated-time true-value false-value)
      (multiple-value-bind (true-value-x true-value-y) (clim:transform-position current-transform mid-x true-value)
	(multiple-value-bind (spoofed-value-x spoofed-value-y) (clim:transform-position current-transform mid-x false-value)
	;; with-draw-options composes the specified transformation with the current transformation
	;; so this leads to the identity transformation
	(clim:with-drawing-options (stream :transformation (clim:invert-transformation current-transform))
	  (clim:draw-circle* stream true-value-x true-value-y 10
			     :filled nil
			     :ink clim:+blue+)
	   (clim:draw-text* stream
			   (format nil "Predicted ~,2f"
				   true-value)
			   (+ true-value-x 20) true-value-y
			   :ink clim:+blue+
			   :align-x :left
			   :align-y :center)
	  (clim:draw-circle* stream spoofed-value-x spoofed-value-y 10
			     :filled nil
			     :ink clim:+red+)
	  (clim:draw-text* stream
			   (format nil "Spoofing detected, time ~,2f." mid-x)
			   (+ spoofed-value-x 20) spoofed-value-y
			   :ink clim:+red+
			   :align-x :left
			   :align-y :center)
	  (clim:draw-text* stream
			   (format nil "Sensor ~,2f" false-value)
			   (+ spoofed-value-x 20) (+ spoofed-value-y line-height)
			   :ink clim:+red+
			   :align-x :left
			   :align-y :center)))))))


;;;; Subordinate Routines for Plotting -- Common Routines

(defmethod draw-axes (left top right bottom
		      x-grid-increment x-n-decimals
		      y-grid-increment y-n-decimals
		      stream)
  ;; draw horizontal lines
  (let* ((label-position (min right (max left 0)))
	 (situation (cond ((= label-position left) :right)
			  ((= label-position right) :left)
			  (t 'neither)))
	 (label-x-attachment (if (eql situation 'neither) :left situation))
	 (label-y-attachment (if (eql situation 'neither) :bottom :center)))
    (loop for y from  bottom upto top by y-grid-increment
	  do (clim:draw-line* stream left y right y
			      :line-thickness 0
			      :line-dashes t
			      :ink *grid-color*
			      )
	     (clim:with-text-size (stream :very-small)
	       (clim:draw-text* stream (format nil "~,v,,,f" y-n-decimals y)
				label-position y
				:align-x label-x-attachment
				:align-y label-y-attachment
				:ink *grid-color*)))
    )
  ;; draw vertical lines
  (let* ((label-position (min top (max bottom 0)))
	 (label-y-attachment (cond ((= label-position bottom) :top)
				   ((= label-position top) :bottom)
				   (t :center)))
	 (label-x-attachment (if (eql label-y-attachment :center)
				 :left :center)))
    (clim:draw-line* stream 0 bottom 0 top
		     :line-thickness 0
		     :line-dashes t
		     :ink *grid-color*
		       )
    (loop for x from left upto right by x-grid-increment
	  do (clim:draw-line* stream x bottom x top
			      :line-thickness 0
			      :line-dashes t
			      :ink *grid-color*)
	     (clim:draw-text* stream (format nil "~,v,,,f" x-n-decimals x)
			      x label-position
			      :text-size :very-small
			      :align-x label-x-attachment
			      :align-y label-y-attachment
			      :ink *grid-color*)))
  )

(defmethod draw-color-key (stream top right-of-grid colors value-names)
  (let ((line-height (clim:stream-line-height stream)))
    ;; first figure out where the starting point is in screen coordinates
    (multiple-value-bind (x y) (clim:transform-position (clim:medium-transformation stream) right-of-grid top)
      ;; (format *error-output* "~%Transformed right of grid ~a" x)
      ;; Now switch to the identify transform so that we can move by line-height easily
      ;; this composes the current transform with the one specified, thereby leading to the identity transformation
      (clim:with-drawing-options (stream :transformation (clim:invert-transformation (clim:medium-transformation stream)))
	(loop with right = (+ x 20)
	    for color in colors
	    for value-name in value-names
	    for y from y by line-height
	    do (clim:draw-line* stream right y (+ right 20) y :ink color :line-thickness 2)
	       (clim:draw-text* stream (string value-name) (+ right 20) y :ink color :align-x :left :align-y :center))))))

(defmethod draw-label (label stream left bottom right line-height)
  (multiple-value-bind (junk line-height)
      (clim:untransform-distance (clim:medium-transformation stream) 0
				 line-height)
    (declare (ignore junk))
    (clim:draw-text* stream label
		     (round (+ right left) 2) (- bottom (abs line-height))
		     :align-x :center :align-y :top
		     :ink *grid-color*)))

(defmethod draw-plot (values stream x-increment &optional (color *plotting-color*))
  (loop for i from 0 below (1- (length values))
      for this-x = 0 then next-x
      for this-y = (aref values i)
      for next-x = (+ this-x x-increment)
      for next-y = (aref values (1+ i))
      do (clim:draw-line* stream this-x this-y next-x next-y
			  :ink color
			  )))

(defun transform-and-y-offset-for-bbox (x-scale y-scale left bottom right top
					&optional (left-margin 0) (bottom-margin 0))
  (declare #+(or allegro genera) (values transform vertical-space)
	   (ignore right))
  (let* ((height (- top bottom))
	 (vertical-space (+ bottom-margin (* y-scale height)))
	 (x-indent (+ left-margin (* x-scale (- left))))
	 (y-indent (+ bottom-margin (* y-scale (- bottom))))
	 (transform (clim:compose-transformations
		      (clim:make-translation-transformation x-indent y-indent)
		      (clim:make-scaling-transformation x-scale y-scale)
		      )))
    (values transform vertical-space)))

(defvar +forest-green+
  #-mcclim (clim-utils::find-named-color "forest-green" (clim:frame-manager-palette (clim:find-frame-manager)))
  #+mcclim clim-internals::+Forestgreen+
        )

(defvar +orange+
  #-mcclim (clim-utils::find-named-color "orange" (clim:frame-manager-palette (clim:find-frame-manager)))
  #+mcclim clim-internals::+orange+)

(defvar +violet+
  #-mcclim (clim-utils::find-named-color "violet" (clim:frame-manager-palette (clim:find-frame-manager)))
  #+mcclim clim-internals::+violet+)

(defvar +navy+
  #-mcclim (clim-utils::find-named-color "navy blue" (clim:frame-manager-palette (clim:find-frame-manager)))
  #+mcclim clim-internals::+navy+)

(defvar +light-blue+
  #-mcclim (clim-utils::find-named-color "light blue" (clim:frame-manager-palette (clim:find-frame-manager)))
  #+mcclim clim-internals::+light-blue+)
