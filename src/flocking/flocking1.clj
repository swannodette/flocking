(ns flocking.flocking1
  (:use [vecmath.vec2 :only [vec2 vec2-zero]])
  (:require [rosado.processing :as p]
            [rosado.processing.applet :as applet]
            [vecmath.core :as vm])
  (:import [processing.core PApplet]))

(def *rnd* (new java.util.Random))
(def *width* 640)
(def *height* 480)
(def *epsilon* (Math/pow 10 -6))
(def aflock (atom []))

(defn limit [v n]
  (vm/mul (vm/unit v) n))

(defn make-boid [loc ms mf]
  {:loc       loc
   :vel       (vec2 (+ (* (.nextFloat *rnd*) 2) -1)
                    (+ (* (.nextFloat *rnd*) 2) -1))
   :acc       (vec2 0 0)
   :r         2.0
   :max-speed ms
   :max-force mf})
 
(defn bound [n ox dx]
  (let [n (int n)
        ox (int ox)
        dx (int dx)]
   (cond 
    (< n (- ox))    (+ dx ox)
    (> n (+ ox dx)) (- ox)
    true n)))

(defn borders [{loc :loc, r :r, :as boid}]
  (assoc boid :loc (vec2 (bound (:x loc) r *width*) (bound (:y loc) r *height*))))
 
(defn render [{[dx dy] :vel, [x y] :loc, r :r, :as boid}]
  (let [dx (float dx)
        dy (float dy)
        r  (float r)
        theta (+ (PApplet/atan2 dy dx) (/ Math/PI 2.0))]
    (p/fill-float 200 100)
    (p/stroke-int 255)
    (p/push-matrix)
    (p/translate x y)
    (p/rotate theta)
    (p/begin-shape TRIANGLES)
    (p/vertex 0 (* (- r) 2.0))
    (p/vertex (- r) (* r 2.0))
    (p/vertex r (* r 2.0))
    (p/end-shape)
    (p/pop-matrix)
    boid))

(defn boids [x y]
  (repeatedly #(make-boid (vec2 x y) 2.0 0.05)))
 
(defn make-flock []
  (let [x (/ *width* 2.0)
        y (/ *height* 2.0)]
   (reset! aflock (into [] (take 150 (boids x y))))))

(declare flock-run)

(defn setup []
  (println "setup")
  (p/framerate 60)
  (make-flock))

(defn draw []
  (println "draw")
  (p/background-int 50)
  (flock-run))
 
(applet/defapplet flocking1 :title "Flocking 1"
  :setup setup :draw draw :size [*width* *height*])
 
(defn steer [{ms :max-speed, mf :max-force, vel :vel, loc :loc, :as boid} target slowdown]
  (let [desired  (vm/sub target loc)
        d        (PApplet/dist (float 0.0)
                               (float 0.0)
                               (float (:x desired))
                               (float (:y desired)))]
    (cond 
     (> d (float 0.0)) (if (and slowdown (< d (float 100.0)))
                         (-> desired
                             vm/unit
                             (vm/mul (* ms (/ d (float 100.0))))
                             (vm/sub vel)
                             (limit mf))
                         (-> desired
                             vm/unit
                             (vm/mul ms)
                             (vm/sub vel)
                             (limit mf)))
     true vec2-zero)))

(defn distance-map
  [{loc :loc, :as boid} boids]
  (map (fn [other] (assoc other :dist (vm/dist (:loc other) loc))) boids))
  
(defn distance-filter
  [boids l u]
  (let [l (float l)
        u (float u)]
   (filter (fn [{d :dist}] (let [d (float d)] (and (> d l) (< d u)))) boids)))
 
(defn vec2-sum
 ([] nil) 
 ([a] a)
 ([a b] (vm/add a b)))

(defn separation-map [{loc :loc :as boid} boids]
  (map (fn [{d :dist oloc :loc}] (-> loc (vm/sub oloc) vm/unit (vm/div d))) boids))
 
(defn separation [boid boids]
  (let [dsep      25.0
        filtered  (distance-filter boids 0.0 dsep)
        final     (separation-map boid filtered)
        acount    (count final)
        sum       (or (and final 
                           (reduce vec2-sum final))
                      vec2-zero)]
    (if (> acount (int 0))
      (vm/div sum acount)
      sum)))

(defn alignment [{mf :max-force :as boid} boids]
  (let [nhood     50.0
        filtered  (distance-filter boids 0 nhood)
        vels      (map :vel filtered)
        acount    (count vels)
        sum       (or (and vels 
                           (reduce vec2-sum vels))
                      vec2-zero)]
    (if (> acount (int 0))
      (limit (vm/div sum acount) mf)
      sum)))
 
(defn cohesion [boid boids]
  (let [nhood      50.0
        filtered   (map :dist (distance-filter boids 0 nhood))
        acount     (count filtered)
        sum        (or (and (> acount (int 0))
                            (reduce vec2-sum filtered))
                       vec2-zero)]
    (if (> acount (int 0))
      (steer boid (vm/div sum acount) nil)
      sum)))
 
(defn flock [{acc :acc, :as boid} boids]
  (let [mboids (distance-map boid boids)
        sep    (-> (separation boid mboids) (vm/mul 2.0))
        ali    (-> (alignment boid mboids) (vm/mul 1.0))
        coh    (-> (cohesion boid mboids) (vm/mul 1.0))]
    (assoc boid :acc (-> acc (vm/add sep) (vm/add ali) (vm/add coh)))))
 
(defn update [{vel :vel, acc :acc, loc :loc, ms :max-speed, :as boid}]
  (assoc boid 
    :vel (limit (vm/add vel acc) ms)
    :loc (vm/add loc vel)
    :acc (vm/mul acc 0.0)))
 
(defn seek [{acc :acc, :as boid} target]
  (assoc boid :acc (steer target nil)))
 
(defn arrive [{acc :acc, :as boid} target]
  (assoc boid :acc (steer target true)))
 
(defn boid-run [boid boids]
  (-> (flock boid boids) update borders))
 
(defn flock-run-all [flock]
  (map #(boid-run % flock) flock))

(defn flock-run []
  (println
   (time
    (do
      (swap! aflock flock-run-all)
      (doseq [boid @aflock]
        (render boid))))))

(comment
  (applet/run flocking1)
  (applet/stop flocking1)
  )

(comment
  ; 3ms normal
  ; 8ms "fast" memoized
  (dotimes [_ 10]
    (let [boids @aflock
          dmap]
     (time
      (dotimes [i 150]
        (dotimes [j 150]
          (fast-boid-dist (nth boids i) (nth boids j)))))))

  (def *test-dists* (dists @aflock))

  ; 7ms
  (dotimes [_ 10]
    (time
     (let [ds (dists @aflock)]
       (dotimes [i 150]
         (println (count (distance-filter (ds (nth @aflock i)) 0.0 25.0)))))))

  ; 4ms this is too slow
  (dotimes [_ 10]
    (time
     (dotimes [_ 150]
       (doall (map #(assoc % :dist 5.0) @aflock)))))

  ; 6ms still really slow
  (dotimes [_ 10]
    (time
     (dotimes [_ 150]
       (loop [x (first @aflock) xs (rest @aflock) result (transient [])]
         (if (nil? x)
           (persistent! result)
           (recur (first xs) (rest xs) (conj! result (assoc x :dist 5.0))))))))

  ; 6ms
  (dotimes [_ 10]
    (time
     (dotimes [_ 150]
       (doall (map :loc @aflock)))))

  ; 2ms
  (dotimes [_ 10]
    (time
     (dotimes [_ 150]
       (doseq [boid @aflock]
         (:loc boid)))))

  (dotimes [_ 10]
    (let [foo {:bar "baz"}]
     (time
      (dotimes [_ (* 150 150)]
        (:bar foo)))))

  ; 2-3ms
  (dotimes [_ 10]
    (let [m {'a 'b 'c 'd 'e 'f 'g 'h 'i 'j}]
        (time
         (dotimes [_ (* 150 150)]
           (assoc m :foo 5.0)))))

  ; < 1ms
  (dotimes [_ 10]
    (let [v1 (:loc (nth @aflock 0))
          v2 (:loc (nth @aflock 1))]
     (time
      (dotimes [_ (* 150 150)]
        (vm/dist v1 v2))))))