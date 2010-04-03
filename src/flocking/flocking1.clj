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

(defn limit [v n]
  (vm/mul (vm/unit v) n))

(def aflock (atom []))

(defn make-boid [loc ms mf]
  {:loc loc
   :vel (vec2 (+ (* (.nextFloat *rnd*) 2) -1)
              (+ (* (.nextFloat *rnd*) 2) -1))
   :acc (vec2 0 0)
   :r  2.0
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
    (fill-float 200 100)
    (stroke-int 255)
    (push-matrix)
    (translate x y)
    (rotate theta)
    (begin-shape TRIANGLES)
    (vertex 0 (* (- r) 2.0))
    (vertex (- r) (* r 2.0))
    (vertex r (* r 2.0))
    (end-shape)
    (pop-matrix)
    boid))
 
(defn make-flock []
  (reset! aflock
          (take 150
                (repeatedly #(make-boid (vec2 (/ *width* 2.0) (/ *height* 2.0)) 2.0 0.05)))))

(declare flock-run)

(defn setup []
  (println "setup")
  (framerate 60)
  (make-flock))

(defn draw []
  (println "draw")
  (background-int 50)
  (flock-run))
 
(defapplet flocking1 :title "Flocking 1"
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
 
(defn loc-map [boids]
  (map #(:loc %) boids))

(defn distance-map [{loc :loc, :as boid} boids]
  (map #(assoc boid :dist (vm/dist % loc)) (loc-map boids)))

(comment
  )
 
(defn distance-map-filter [dmap l u]
  (filter (fn [[x]]
            (let [x (float x)
                  l (float l)
                  u (float u)]
             (and (> x l) 
                  (< x u))))
          dmap))
 
(defn separation-map [loc dm]
  (map (fn [[v1 v2]] (-> loc (sub v2) unit (div v1))) dm))
 
(defn separate [{loc :loc, :as boid} boids]
  (let [dsep      25.0
        dm        (distance-map boid boids)
        filtered  (distance-map-filter dm 0.0 dsep)
        final     (separation-map loc filtered)
        acount    (count final)
        sum       (or (and final 
                           (reduce add final))
                      vec2-zero)]
    (if (> acount (int 0))
      (div sum acount)
      sum)))
 
(defn dist-vel-loc-map [{loc :loc, :as boid} boids]
  (let [lx (float (:x loc))
        ly (float (:y loc))]
    (map (fn [{oloc :loc vel :vel :as b}]
          (let [x (float (:x oloc))
                y (float (:y oloc))
                vel (float (:vel b))]
            (vector (PApplet/dist x y lx ly) vel)))
        boids)))
 
(defn align [{mf :max-force, loc :loc, :as boid} boids]
  (let [nhood     50.0
        dvl       (dist-vel-loc-map boid boids)
        filtered  (distance-map-filter dvl 0 nhood)
        vels      (map (fn [[_ vel]] vel) filtered)
        acount    (count vels)
        sum       (or (and vels 
                           (reduce add vels))
                      vec2-zero)]
    (if (> acount (int 0))
      (limit (div sum acount) mf)
      sum)))
 
(defn cohesion [{loc :loc, :as boid} boids]
  (let [nhood      50.0
        dm         (distance-map boid boids)
        filtered   (distance-map-filter dm 0 nhood)
        acount     (count filtered)
        sum        (or (and (> acount (int 0))
                            (reduce add (map (fn [[_ n]] n) filtered)))
                       vec2-zero)]
    (if (> acount (int 0))
      (steer boid (div sum acount) nil)
      sum)))
 
(defn flock [{acc :acc, :as boid} boids]
  (let [sep (-> (separate boid boids) (mult 2.0))
        ali (-> (align boid boids) (mult 1.0))
        coh (-> (cohesion boid boids) (mult 1.0))]
    (assoc boid :acc (map + acc sep ali coh))))
 
(defn update [{vel :vel, acc :acc, loc :loc, ms :max-speed, :as boid}]
  (assoc boid 
    :vel (limit (add vel acc) ms)
    :loc (add loc vel)
    :acc (mult acc 0.0)))
 
(defn seek [{acc :acc, :as boid} target]
  (assoc boid :acc (steer target nil)))
 
(defn arrive [{acc :acc, :as boid} target]
  (assoc boid :acc (steer target true)))
 
(defn boid-run [boid boids]
  (borders (update (flock boid boids))))
 
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
  (run flocking1))