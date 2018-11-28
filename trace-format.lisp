;;; -*- Mode: Commonlisp; package: Awdrat; Readtable: Joshua -*-

(in-package :awdrat)

(defparameter *do-tracing* nil)

(defmacro trace-format (control-string &rest args)
  `(when *do-tracing*
     (format *error-output* ,control-string ,@args)))
