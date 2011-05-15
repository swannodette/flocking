(ns flocking.perf
  (:use [flocking.flocking]))

(make-flock)

; 10-12ms vs 6ms-8ms in Processing
(dotimes [_ 100]
  (time
   (reset! aflock (doall (flock-run-all @aflock)))))

; 110-120ms vs 34-35ms in Processing
(make-flock 500)
(dotimes [_ 100]
  (time
   (reset! aflock (doall (flock-run-all @aflock)))))

; 1.1ms
(dotimes [_ 100]
  (let [b  (nth @aflock 0)
        bs (distance-map b @aflock)]
   (time
    (doseq [b bs]
      (separation b bs)))))

; 1.1ms
(dotimes [_ 100]
  (let [b  (nth @aflock 0)
        bs (distance-map b @aflock)]
   (time
    (doseq [b bs]
      (alignment b bs)))))

; 1.1ms
(dotimes [_ 100]
  (let [b  (nth @aflock 0)
        bs (distance-map b @aflock)]
   (time
    (doseq [b bs]
      (cohesion b bs)))))

; 7ms
(dotimes [_ 100]
  (let [bs @aflock]
   (time
    (doseq [b bs]
      (doall (distance-map b bs))))))