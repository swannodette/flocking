(ns flocking.flocking1
  (:use [rosado.processing]
        [rosado.processing.applet])
  (:import [processing.core PApplet]))

(def *rnd* (new java.util.Random))
(def *width* 640)
(def *height* 480)
(def *epsilon* (Math/pow 10 -6))

(defn equal [[x1 y1] [x2 y2]]
  (and (< (- x1 x2) *epsilon*)
       (< (- y1 y2) *epsilon*)))

(defn add [[x1 y1] [x2 y2]]
  [(+ (float x1) (float x2))
   (+ (float y1) (float y2))])
 
(defn sub [[x1 y1] [x2 y2]]
  [(- (float x1) (float x2))
   (- (float y1) (float y2))])

(defn mult [[x y] s]
  [(* (float x) (float s)) (* (float y) (float s))])
 
(defn div [v s]
  (mult v (/ 1.0 (float s))))

(defn unit [[x y]]
  (let [x (float x)
        y (float y)
        d (PApplet/dist 0.0 0.0 x y)]
    [(/ x d) (/ y d)]))

(defn limit [v n]
  (mult (unit v) n))

(def aflock (atom []))

(defn make-boid [loc ms mf]
  {:loc loc
   :vel [(+ (* (.nextFloat *rnd*) 2) -1)
         (+ (* (.nextFloat *rnd*) 2) -1)]
   :acc [0 0]
   :r  2.0
   :max-speed ms
   :max-force mf})
 
(defn bound [n ox dx]
  (let [n (int n)
        ox (int ox)
        dx (int dx)]
   (cond 
    (< n (- ox)) (+ dx ox)
    (> n (+ ox dx)) (- ox)
    true n)))

(defn borders [{[x y] :loc, r :r, :as boid}]
  (assoc boid :loc [(bound x r *width*) (bound y r *height*)]))
 
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
                (repeatedly #(make-boid [(/ *width* 2.0) (/ *height* 2.0)] 2.0 0.05)))))

(declare flock-run)

(defn setup []
  (println "setup")
  (framerate 60)
  (make-flock)0)

(defn draw []
  (println "draw")
  (background-int 50)
  (flock-run))
 
(defapplet flocking1 :title "Flocking 1"
  :setup setup :draw draw :size [*width* *height*])
 
(defn steer [{ms :max-speed, mf :max-force, vel :vel, loc :loc, :as boid} target slowdown]
  (let [[desiredx desiredy :as desired]  (sub target loc)
        d                                (PApplet/dist 0.0 0.0 (float desiredx) (float desiredy))]
    (cond 
     (> d 0.0) (if (and slowdown (< d 100.0))
                 (limit (sub (mult (unit desired) (* ms (/ d 100.0))) vel) mf)
                 (limit (sub (mult (unit desired) ms) vel) mf))
     true [0.0 0.0])))
 
(defn loc-map [boids]
  (map #(:loc %) boids))

(defn distance-map [{[lx ly] :loc, :as boid} boids]
  (map (fn [[x1 y1 :as loc]]
         (let [x1 (float x1)
               y1 (float y1)
               lx (float lx)
               ly (float ly)]
          (vector (PApplet/dist x1 y1 lx ly) loc)))
       (loc-map boids)))
 
(defn distance-map-filter [dmap l u]
  (filter (fn [[x]]
            (let [x (float x)
                  l (float l)
                  u (float u)]
             (and (> x l) 
                  (< x u))))
          dmap))
 
(defn separation-map [loc dm]
  (map (fn [[v1 v2]] (div (unit (sub loc v2)) v1)) dm))
 
(defn separate [{loc :loc, :as boid} boids]
  (let [dsep      25.0
        dm        (distance-map boid boids)
        filtered  (distance-map-filter dm 0.0 dsep)
        final     (separation-map loc filtered)
        acount    (count final)
        sum       (or (and final 
                           (reduce add final))
                      [0.0 0.0])]
    (if (> acount 0)
      (div sum acount)
      sum)))
 
(defn dist-vel-loc-map [{[lx ly] :loc, :as boid} boids]
  (let [lx (float lx)
        ly (float ly)]
   (map (fn [{[x y :as loc] :loc vel :vel :as b}]
          (let [x (float x)
                y (float y)
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
                      [0.0 0.0])]
    (if (> acount 0)
      (limit (div sum acount) mf)
      sum)))
 
(defn cohesion [{loc :loc, :as boid} boids]
  (let [nhood      50.0
        dm         (distance-map boid boids)
        filtered   (distance-map-filter dm 0 nhood)
        acount     (count filtered)
        sum        (or (and (> acount 0)
                            (reduce add (map (fn [[_ n]] n) filtered)))
                       [0.0 0.0])]
    (if (> acount 0)
      (steer boid (div sum acount) nil)
      sum)))
 
(defn flock [{acc :acc, :as boid} boids]
  (let [sep (mult (separate boid boids) 2.0)
        ali (mult (align boid boids) 1.0)
        coh (mult (cohesion boid boids) 1.0)]
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