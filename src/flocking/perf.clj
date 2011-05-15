(ns flocking.perf
  (:use [flocking.flocking]))

(make-flock)

; 10-12ms vs 6ms-8ms in Processing
(dotimes [_ 10]
  (time
   (reset! aflock (doall (flock-run-all)))))

; 70ms vs 34-35ms in Processing
(make-flock 500)
(dotimes [_ 10]
  (time
   (reset! aflock (doall (flock-run-all)))))

;; < 1ms
(dotimes [_ 10]
  (let [b  (get-in @aflock [0 0])
        bs (distance-map b (whole-flock))]
   (time
    (doseq [b bs]
      (separation b bs)))))

(comment
  (let [b  (get-in @aflock [0 0])
        bs (distance-map b (whole-flock))]
    bs)
  )

;; < 1ms
(dotimes [_ 10]
  (let [b  (get-in @aflock [0 0])
        bs (distance-map b (whole-flock))]
   (time
    (doseq [b bs]
      (alignment b bs)))))

; < 1ms
(dotimes [_ 100]
  (let [b  (get-in @aflock [0 0])
        bs (distance-map b (whole-flock))]
   (time
    (doseq [b bs]
      (cohesion b bs)))))

; 10ms
(dotimes [_ 10]
  (let [flock (whole-flock)]
   (time
    (doseq [b flock]
      (doall (distance-map b flock))))))