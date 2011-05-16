(ns flocking.flocking
  (:refer-clojure :exclude [zero?])
  (:require [rosado.processing :as p]
            [rosado.processing.applet :as applet]))

(set! *unchecked-math* true)
(set! *warn-on-reflection* true)

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
  (Vec2d. (/ (.x v) scalar) (/ (.y v) scalar)))

(def ^Vec2d zero (Vec2d. 0.0 0.0))

(defn ^Vec2d sum
  ([] zero)
  ([v1 v2] (add v1 v2)))

(defn zero? [v]
  (identical? v zero))

;; =============================================================================
;; Top-level values
;; =============================================================================

(def ^java.util.Random rnd (new java.util.Random))
(def ^:constant width 640.0)
(def ^:constant height 360.0)
(def ^:constant boid-count 500)
(def ^:constant cores (.. Runtime getRuntime availableProcessors))
(def aflock (atom nil))

;; =============================================================================
;; Utilities
;; =============================================================================

(defn limit [v ^double n]
  (mul (unit v) n))

(comment
  (defprotocol IBoid
    (steer [this target slowdown])
    (cohere [this flock])
    (separate [this flock])
    (align [this flock])
    (update [this]))
  )

(defn bound ^double [^double n ^double ox ^double dx]
  (cond 
   (< n (- ox)) (+ dx ox)
   (> n (+ ox dx)) (- ox)
   true n))

(defprotocol IBoid
  (steer [this target slowdown])
  (borders [this])
  (update [this])
  (flock [this flock]))

(declare distance-map)
(declare seperation)
(declare alignment)
(declare cohesion)

(defrecord Boid [^Vec2d loc
                 ^Vec2d vel
                 ^Vec2d acc
                 ^double r
                 ^double max-speed
                 ^double max-force]
  IBoid
  (steer [_ target slowdown]
         (let [ms max-speed
               mf max-force
               desired (sub target loc)
               d (length desired)]
           (cond 
            (> d 0.0) (let [unit (unit desired)]
                        (-> (if (and slowdown (< d 100.0))
                              (-> unit (mul (* ms (/ d 100.0))))
                              (-> unit (mul ms)))
                            (sub vel)
                            (limit mf)))
            :else zero)))
  (borders [this]
           (assoc this :loc (Vec2d. (bound (.x loc) r width) (bound (.y loc) r height))))
  (update [this]
          (assoc this
            :vel (limit (add vel acc) max-speed)
            :loc (add loc vel)
            :acc (mul acc 0.0)))
  (flock [this boids]
         (let [mboids (distance-map this boids)
               sep (-> (separation this mboids) (mul 2.0))
               ali (-> (alignment this mboids) (mul 1.0))
               coh (-> (cohesion this mboids) (mul 1.0))]
           (assoc this :acc (-> acc (add sep) (add ali) (add coh))))))

(defrecord DistBoid [^Vec2d loc
                     ^Vec2d vel
                     ^Vec2d acc
                     ^double r
                     ^double max-speed
                     ^double max-force
                     ^double dist])

(defn ^Boid make-boid [loc ms mf]
  (Boid. loc
         (Vec2d. (+ (* (.nextDouble rnd) 2.0) -1.0)
                 (+ (* (.nextDouble rnd) 2.0) -1.0))
         (Vec2d. 0.0 0.0)
         2.0 ms mf))

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

(defn distance-map [{bloc :loc} boids]
  (map (fn [^Boid other]
         (let [d (length (sub (.loc other) bloc))]
           (DistBoid. (.loc other)
                      (.vel other)
                      (.acc other)
                      (.r other)
                      (.max-speed other)
                      (.max-force other)
                      d)))
       boids))

(defn distance-filter [boids ^double l ^double u]
  (filter (fn [^DistBoid other] (let [d (.dist other)] (and (> d l) (< d u)))) boids))

(defn separation-map [{loc :loc :as boid} boids]
  (map (fn [^DistBoid other]
         (let [d (.dist other)
               oloc (.loc other)]
          (-> loc (sub oloc) unit (div d))))
       boids))

(defn separation
  [boid boids]
  (let [dsep 25.0
        filtered (separation-map boid (distance-filter boids 0.0 dsep))]
    (let [sum (reduce sum filtered)]
      (if (not (zero? sum))
        (div sum (count filtered))
        sum))))

(defn alignment
  [{mf :max-force :as boid} boids]
  (let [mf (double mf)
        nhood 50.0
        filtered (map #(.vel ^DistBoid %) (distance-filter boids 0.0 nhood))]
    (let [sum (reduce sum filtered)]
      (if (not (zero? sum))
        (limit (div sum (count filtered)) mf)
        sum))))

(defn cohesion [boid boids]
  (let [nhood 50.0
        filtered (map #(.loc ^DistBoid %) (distance-filter boids 0.0 nhood))]
    (let [sum (reduce sum filtered)]
      (if (not (zero? sum))
       (steer boid (div sum (count filtered)) false)
       sum))))

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

;; for testing purposes
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