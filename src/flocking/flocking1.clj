(ns flocking.flocking1
  (:use [vecmath.vec2 :only [vec2 zero sum]]
        clojure.contrib.pprint)
  (:require [rosado.processing :as p]
            [rosado.processing.applet :as applet]
            [vecmath.core :as vm]))

(deftype dist-boid
  [loc vel acc r max-speed max-force dist]
  clojure.lang.IPersistentMap)

(def #^java.util.Random *rnd* (new java.util.Random))
(def *width* 640)
(def *height* 360)
(def *boid-count* 150)
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
  (let [n  (int n)
        ox (int ox)
        dx (int dx)]
   (cond 
    (< n (int (- ox)))    (+ dx ox)
    (> n (int (+ ox dx))) (- ox)
    true n)))

(defn borders [{loc :loc, r :r, :as boid}]
  (assoc boid :loc (vec2 (bound (:x loc) r *width*) (bound (:y loc) r *height*))))
 
(defn render [{{dx :x dy :y} :vel, {x :x y :y} :loc, r :r, :as boid}]
  (let [dx (float dx)
        dy (float dy)
        r  (float r)
        theta (float (+ (float (p/atan2 dy dx))
                        (float (/ (float Math/PI) (float 2.0)))))]
    (p/fill-float 200 100)
    (p/stroke-int 255)
    (p/push-matrix)
    (p/translate x y)
    (p/rotate theta)
    (p/begin-shape :triangles)
    (p/vertex 0 (* (- r) 2.0))
    (p/vertex (- r) (* r 2.0))
    (p/vertex r (* r 2.0))
    (p/end-shape)
    (p/pop-matrix)))

(defn boids [x y]
  (repeatedly #(make-boid (vec2 x y) 2.0 0.05)))
 
(defn make-flock []
  (let [x (/ *width* 2.0)
        y (/ *height* 2.0)]
   (reset! aflock (into [] (take *boid-count* (boids x y))))))

(declare flock-run)

(defn setup []
  (p/smooth)
  (make-flock))

(defn draw []
  (p/background-int 50)
  (flock-run))
 
(applet/defapplet flocking1 :title "Flocking 1"
  :setup setup :draw draw :size [*width* *height*])
 
(defn steer [{ms :max-speed, mf :max-force, vel :vel, loc :loc, :as boid} target slowdown]
  (let [{x :x y :y :as desired} (vm/sub target loc)
        d                       (float (p/dist (float 0.0) (float 0.0) (float x) (float y)))]
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
     true zero)))

(defn distance-map
  [boid boids]
  (let [bloc (:loc boid)]
    (map (fn [other]
           (let [loc (:loc other)]
            (dist-boid loc
                       (:vel other)
                       (:acc other)
                       (:r other)
                       (:max-speed other)
                       (:max-force other)
                       (vm/dist loc bloc)))) boids)))

(defn distance-filter
  [boids l u]
  (let [l (float l)
        u (float u)]
   (filter (fn [other] (let [d (float (:dist other))] (and (> d l) (< d u)))) boids)))

(defn separation-map [{loc :loc :as boid} boids]
  (map (fn [other]
         (let [d (:dist other)
               oloc (:loc other)]
          (-> loc (vm/sub oloc) vm/unit (vm/div d))))
       boids))
 
(defn separation
  [boid boids]
  (let [dsep     25.0
        filtered (separation-map boid (distance-filter boids 0.0 dsep))]
    (if-let [sum (reduce sum filtered)]
      (vm/div sum (count filtered))
      zero)))

(defn alignment
  [{mf :max-force :as boid} boids]
  (let [nhood    50.0
        filtered (map :vel (distance-filter boids 0 nhood))]
    (if-let [sum (reduce sum filtered)]
      (limit (vm/div sum (count filtered)) mf)
      zero)))
 
(defn cohesion
  [boid boids]
  (let [nhood    50.0
        filtered (map :loc (distance-filter boids 0 nhood))]
    (if-let [sum (reduce sum filtered)]
      (steer boid (vm/div sum (count filtered)) false)
      zero)))
 
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

(defn boid-run [boid boids]
  (-> (flock boid boids) update borders))
 
(defn flock-run-all [flock]
  (map #(boid-run % flock) flock))

(defn flock-run []
  (swap! aflock flock-run-all)
  (doseq [boid @aflock]
    (render boid)))