(ns flocking.flocking-old
  (:use rosado.processing sketch cljos)
  (:import [processing.core PApplet PVector]))

(def *boid-count* 150)

(defmacro pvector [a b]
  `(new PVector (float ~a) (float ~b)))

(def aflock (ref []))
(defclass boid [object] :loc :vel :acc :r :max-force :max-speed)

(def get-loc (accessor boid :loc))
(def get-vel (accessor boid :vel))

;; have to define getters, otherwise reflection kicks in
(defn getX [#^PVector v]
  (. v x))

(defn getY [#^PVector v]
  (. v y))

(defmacro vmult [#^PVector v s]
  `(PVector/mult ~v (float ~s)))

(defmacro vdiv [#^PVector v s]
  `(PVector/div ~v (float ~s)))

(defn #^PVector vadd [#^PVector v1 #^PVector v2]
  (PVector/add v1 v2))

(defn #^PVector vsub [#^PVector v1 #^PVector v2]
  (PVector/sub v1 v2))

(defn #^PVector vunit [#^PVector v]
  (.normalize v (pvector 0 0)))

(defn #^Float/TYPE vdist [#^PVector v1 #^PVector v2]
  (PVector/dist v1 v2))

(defn #^Float/TYPE vdist-zero [#^PVector v]
  (PVector/dist (pvector 0 0) v))

(defn #^floats distances [#^PVector pvector #^"[Lprocessing.core.PVector;" pvarray]
  (let [len (int (count pvarray))]
     (loop [x (int 0) #^floats result (make-array Float/TYPE len)]
       (if (> x (dec len))
	 result
	 (do
	   (aset-float result x (float (vdist pvector (aget pvarray x))))
	   (recur (unchecked-inc x) result))))))

(defn #^PVector limit [#^PVector v n]
  (vmult (vunit v) n))

(defn bound [n ox dx]
  (cond 
   (< n (- ox)) (+ dx ox)
   (> n (+ ox dx)) (- ox)
   true n))

(defn borders [{#^PVector loc :loc, r :r, :as boid}]
    (assoc boid 
      :loc (pvector (bound (getX loc) r 640) 
		    (bound (getY loc) r 480))))

(defn render [{#^PVector vel :vel, #^PVector loc :loc, r :r, :as boid}]
  (let [theta (+ (atan2 (getY vel) (getX vel)) (float (/ Math/PI 2)))]
    (fill-float 200 100)
    (stroke-int 255)
    (push-matrix)
    (translate (getX loc) (getY loc))
    (rotate (float theta))
    (begin-shape :triangles)
    (vertex 0 (* (- r) 2.0))
    (vertex (- r) (* r 2.0))
    (vertex r (* r 2.0))
    (end-shape)
    (pop-matrix)
    boid))

(defn init-boid [boid, l, ms, mf]
  (assoc boid 
    :acc (pvector 0 0)
    :vel (pvector (random -1 1) (random -1 1))
    :loc l
    :r   (float 2.0)
    :max-speed ms
    :max-force mf))

(defn make-flock []
  (dosync
   (dotimes [x *boid-count*]
     (commute aflock conj 
	      (init-boid (make-instance boid)
			 (pvector 320 240) 
			 2.0 
			 0.05)))))

(delcare flock-run)

(defsetup [dst]
  (size 640 480)
  (smooth)
  (make-flock))

(defdraw [dst]
  (background-int 50)
  (flock-run))

(defn #^PVector steer [{ms :max-speed, 
			mf :max-force, 
			#^PVector vel :vel, 
			#^PVector loc :loc, :as boid} 
		        #^PVector target 
			slowdown]
  (let [#^PVector desired    (vsub target loc)
	d                    (float (vdist-zero desired))]
    (cond 
     (> d 0.0) (if (and slowdown (< d 100.0))
	       (limit (vsub 
		       (vmult (vunit desired) (* ms (/ d 100.0))) vel) mf)
	       (limit (vsub 
		       (vmult (vunit desired) ms) vel) mf))
     true (pvector 0 0))))

(defn loc-map [boids]
  (map get-loc boids))

(defn vel-map [boids]
  (map get-vel boids))

(defn #^PVector separation
  [{#^PVector loc :loc, :as boid} 
    #^floats dists 
    #^"[Lprocessing.core.PVector;" locs]
  (let [dsep             (float 25.0)
	len              (int (dec *boid-count*))
	[#^PVector sum 
	 acount]         (loop [i (int 0) rcount (int 0) #^PVector result (pvector 0 0)]
			   (if (> i len)
			     [result rcount]
 			     (let [d                 (float (aget dists i))
				   oloc              (aget locs i)
				   inhood            (and (> d (float 0))
							  (< d dsep))
				   ncount            (int (if inhood (unchecked-inc rcount) rcount))
 				   #^PVector nresult (if inhood 
						       (vadd result 
							     (vdiv (vunit (vsub loc oloc)) d))
						       result)]
			       (recur (unchecked-inc i) ncount nresult))))]
    (if (> (int acount) (int 0))
	(vdiv sum acount)
	sum)))

(defn #^PVector alignment
  [{mf :max-force, #^PVector loc :loc, :as boid} 
    #^floats dists 
    #^"[Lprocessing.core.PVector;" vels]
  (let [nhood            (float 50.0)
	len              (int (dec *boid-count*))
	[#^PVector sum 
	 acount]         (loop [i (int 0) rcount (int 0) #^PVector result (pvector 0 0)]
			   (if (> i len)
			     [result rcount]
			     (let [d                 (float (aget dists i))
				   #^PVector vel     (aget vels i)
				   inhood            (and (> d (float 0))
							  (< d nhood))
				   ncount            (int (if inhood 
							    (unchecked-inc rcount) 
							    rcount))
				   #^PVector nresult (if inhood 
						       (vadd result vel)
						       result)]
			       (recur (unchecked-inc i) ncount nresult))))]
    (if (> (int acount) (int 0))
      (limit (vdiv sum acount) mf)
      sum)))

(defn #^PVector cohesion 
  [{#^PVector loc :loc, :as boid} 
    #^floats dists 
    #^"[Lprocessing.core.PVector;" locs]
  (let [nhood          (float 50.0)
	len            (int (dec *boid-count*))
    	[#^PVector sum 
	 acount]       (loop [i (int 0) rcount (int 0) result (pvector 0 0)]
			 (if (> i len)
			   [result rcount]
			   (let [d                 (float (aget dists i))
				 #^PVector oloc    (aget locs i)
				 inhood            (and (> d (float 0))
							(< d nhood))
				 ncount            (int (if inhood (unchecked-inc rcount) rcount))
				 #^PVector nresult (if inhood 
						     (vadd result oloc)
						     result)]
			     (recur (unchecked-inc i) ncount nresult))))]
    (if (> (int acount) (int 0))
      (steer boid (vdiv sum acount) nil)
      sum)))

(defn #^"[Lprocessing.core.PVector;" toPVectorArray 
  [avector]
  (let [len (count avector)]
    (loop [x 0 result (make-array PVector len)]
      result
      (if (> x (dec len))
	result
	(do
	  (aset result x (nth avector x))
	  (recur (unchecked-inc x) result))))))

(defn flock [{#^PVector loc :loc, #^PVector acc :acc, :as boid} 
	     #^"[Lprocessing.core.PVector;" locs 
	     #^"[Lprocessing.core.PVector;" vels]
  (let [#^floats  dists (distances loc locs)
	#^PVector sep   (vmult (separation boid dists locs) 2.0)
	#^PVector ali   (vmult (alignment boid dists vels) 1.0)
	#^PVector coh   (vmult (cohesion boid dists locs) 1.0)]
    (assoc boid :acc (vadd (vadd (vadd acc sep) ali) coh))))

(defn update [{#^PVector vel :vel, #^PVector acc :acc, #^PVector loc :loc, ms :max-speed, :as boid}]
     (assoc boid 
       :vel (limit (vadd vel acc) ms)
       :loc (vadd loc vel)
       :acc (vmult acc 0)))

(defn boid-run [boid locs vels]
  (borders (update (flock boid locs vels))))

(defn flock-run-all [flock]
  (let [locs     (toPVectorArray (loc-map flock))
	vels     (toPVectorArray (vel-map flock))
	len      (dec (count flock))]
      (loop [i 0 result []]
        (if (> i len)
  	result
  	(recur (unchecked-inc i) 
  	       (conj result (boid-run (nth flock i) locs vels)))))))

(defn flock-run []
  (dosync
   (commute aflock flock-run-all))
  (doseq [boid @aflock]
    (render boid)))