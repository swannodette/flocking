(ns flocking.test-flocking1
  (:use [flocking.flocking1] :reload-all)
  (:use [clojure.test]))

(deftest test-add
  (is (equal [5.0 2.5] (add [2.5 1.1] [2.5 1.4]))))

(deftest test-sub
  (is (equal [2.5 2.30] (sub [5.0 7.30] [2.50 5.0]))))

(deftest test-mult
  (is (equal (mult [2.5 3.3] 2) [5.0 6.6])))

(deftest test-div
  (is (equal (div [6.66 333.3] 3) [2.22 111.1])))