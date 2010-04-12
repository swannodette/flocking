(ns flocking.utils)

(defmacro conj-time!
  [atom expr]
  `(let [start# (. System (nanoTime))
         ret# ~expr]
     (swap! ~atom conj (str "Elapsed time: " (/ (double (- (. System (nanoTime)) start#)) 1000000.0) " msecs"))
     ret#))