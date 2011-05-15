(ns flocking.flocking
  (:require [rosado.processing :as p]
            [rosado.processing.applet :as applet]))

(set! *unchecked-math* true)

(defprotocol IVecMath2d
  (add [this other])
  (sub [this other])
  (unit [this]))

(declare length)

(deftype Vec2d [^double x ^double y]
  IVecMath2d
  (add [_ other]
       (let [o ^Vec2d other]
         (Vec2d. (+ x (.x o)) (+ y (.y o)))))
  (sub [_ other]
       (let [o ^Vec2d other]
         (Vec2d. (- x (.x o)) (- y (.y o)))))
  (unit [this]
        (let [d (length this)]
          (Vec2d. (/ x d) (/ y d))))
  Object
  (toString [this]
            (str "<" x ", " y ">")))

(defmethod print-method Vec2d [v ^java.io.Writer writer]
  (.write writer (str v)))

(defn length ^double [^Vec2d v]
  (let [x (.x v)
        y (.y v)]
    (Math/sqrt (+ (* x x) (* y y)))))

(defn ^Vec2d mul [^Vec2d v ^double scalar]
  (Vec2d. (* (.x v) scalar) (* (.y v) scalar)))

(defn ^Vec2d div [^Vec2d v ^double scalar]
  (Vec2d. (* (.x v) scalar) (* (.y v) scalar)))

(def ^Vec2d zero (Vec2d. 0.0 0.0))

(defn ^Vec2d sum
  ([] zero)
  ([v1 v2] (add v1 v2)))

;; =============================================================================
;; Top-level values
;; =============================================================================

(def ^java.util.Random rnd (new java.util.Random))
(def ^:constant width 640.0)
(def ^:constant height 360.0)
(def ^:constant boid-count 150)
(def ^:constant cores (.. Runtime getRuntime availableProcessors))
(def aflock (atom nil))

;; =============================================================================
;; Utilities
;; =============================================================================

(defn limit [v n]
  (mul (unit v) n))

(defn make-boid [loc ms mf]
  {:loc loc
   :vel (Vec2d. (+ (* (.nextDouble rnd) 2) -1)
                (+ (* (.nextDouble rnd) 2) -1))
   :acc (Vec2d. 0 0)
   :r 2.0
   :max-speed ms
   :max-force mf})

(defn bound ^double [^double n ^double ox ^double dx]
  (cond 
   (< n (- ox))    (+ dx ox)
   (> n (+ ox dx)) (- ox)
   true n))

(defn borders [{^Vec2d loc :loc, r :r, :as boid}]
  (assoc boid :loc (Vec2d. (bound (.x loc) r width) (bound (.y loc) r height))))

(defn make-flock
  ([] (make-flock boid-count))
  ([n]
     (let [x (/ width 2.0)
           y (/ height 2.0)
           m (+ 2 cores)]
       (reset! aflock
               (into []
                     (for [i (range m)]
                       (into []
                             (for [j (range (/ n m))]
                               (make-boid (Vec2d. x y) 2.0 0.05)))))))))

;; =============================================================================
;; Flocking
;; =============================================================================

(defn steer [{ms :max-speed, mf :max-force, vel :vel, loc :loc, :as boid} target slowdown]
  (let [ms (double ms)
        mf (double mf)
        desired (sub target loc)
        d (length desired)]
    (cond 
     (> d 0.0) (let [unit (unit desired)]
                 (-> (if (and slowdown (< d 100.0))
                       (-> unit (mul (* ms (/ d 100.0))))
                       (-> unit (mul ms)))
                     (sub vel)
                     (limit mf)))
     true zero)))

(defn distance-map
  [boid boids]
  (let [bloc (:loc boid)]
    (map (fn [other]
           (let [loc (:loc other)]
             (assoc other :dist (length (sub loc bloc))))) boids)))

(defn distance-filter
  [boids ^double l ^double u]
  (filter (fn [other] (let [d (:dist other)] (and (> d l) (< d u)))) boids))

(defn separation-map [{loc :loc :as boid} boids]
  (map (fn [other]
         (let [d (:dist other)
               oloc (:loc other)]
           (-> loc (sub oloc) unit (div d))))
       boids))

(defn separation
  [boid boids]
  (let [dsep     25.0
        filtered (separation-map boid (distance-filter boids 0.0 dsep))]
    (if-let [sum (reduce sum filtered)]
      (div sum (count filtered))
      zero)))

(defn alignment
  [{mf :max-force :as boid} boids]
  (let [nhood    50.0
        filtered (map :vel (distance-filter boids 0.0 nhood))]
    (if-let [sum (reduce sum filtered)]
      (limit (div sum (count filtered)) mf)
      zero)))

(defn cohesion
  [boid boids]
  (let [nhood    50.0
        filtered (map :loc (distance-filter boids 0.0 nhood))]
    (if-let [sum (reduce sum filtered)]
      (steer boid (div sum (count filtered)) false)
      zero)))

(defn flock [{acc :acc, :as boid} boids]
  (let [mboids (distance-map boid boids)
        sep    (-> (separation boid mboids) (mul 2.0))
        ali    (-> (alignment boid mboids) (mul 1.0))
        coh    (-> (cohesion boid mboids) (mul 1.0))]
    (assoc boid :acc (-> acc (add sep) (add ali) (add coh)))))

(defn update [{vel :vel, acc :acc, loc :loc, ms :max-speed, :as boid}]
  (assoc boid 
    :vel (limit (add vel acc) ms)
    :loc (add loc vel)
    :acc (mul acc 0.0)))

(defn boid-run [boid boids]
  (-> (flock boid boids) update borders))

(defn subflock-run [subflock flock]
  (into [] (map #(boid-run % flock) subflock)))

(defn update-flock [flock current]
  (into [] (pmap #(subflock-run % current) flock)))

(declare render)

;; for testing purposes
(defn flock-run-all []
  (let [flock (into [] (apply concat @aflock))]
    (update-flock @aflock flock)))

(defn whole-flock []
  (into [] (apply concat @aflock)))

(defn flock-run []
  (let [flock (into [] (apply concat @aflock))]
    (swap! aflock update-flock flock)
    (doseq [boid flock]
      (render boid))))

;; =============================================================================
;; Applet
;; =============================================================================

(defn render [{:keys [^Vec2d vel ^Vec2d loc r] :as boid}]
  (let [r (double r)
        dx (.x vel)
        dy (.y vel)
        x (.x loc)
        y (.y loc)
        theta (+ (p/atan2 dy dx)
                 (/ Math/PI 2.0))]
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

(defn setup []
  (p/smooth)
  (make-flock))

(defn draw []
  (p/background-int 50)
  (flock-run))

(applet/defapplet flocking :title "Flocking"
  :setup setup :draw draw :size [width height])