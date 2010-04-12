(ns flocking.perf
  (:require [flocking.flocking1 :as f0]
            [flocking.flocking1 :as f1]))

(f0/make-flock)

; 10-12ms vs 6ms-8ms in Processing
(dotimes [_ 100]
  (time
   (reset! f0/aflock (doall (f0/flock-run-all @f0/aflock)))))

(f1/make-flock)

; 10-12ms vs 6ms-8ms in Processing
(dotimes [_ 100]
  (time
   (reset! f1/aflock (doall (f1/flock-run-all @f1/aflock)))))

; 1.1ms
(dotimes [_ 100]
  (let [b  (nth @f1/aflock 0)
        bs (f1/distance-map b @f1/aflock)]
   (time
    (doseq [b bs]
      (f1/separation b bs)))))

; 1.1ms
(dotimes [_ 100]
  (let [b  (nth @f1/aflock 0)
        bs (f1/distance-map b @f1/aflock)]
   (time
    (doseq [b bs]
      (f1/alignment b bs)))))

; 1.1ms
(dotimes [_ 100]
  (let [b  (nth @f1/aflock 0)
        bs (f1/distance-map b @f1/aflock)]
   (time
    (doseq [b bs]
      (f1/cohesion b bs)))))

; 7ms
(dotimes [_ 100]
  (let [bs @f1/aflock]
   (time
    (doseq [b bs]
      (doall (f1/distance-map b bs))))))