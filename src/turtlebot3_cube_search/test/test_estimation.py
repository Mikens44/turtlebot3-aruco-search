import numpy as np
import pytest

from turtlebot3_cube_search.estimation import CubeEstimator


def estimator(**kwargs):
    return CubeEstimator(min_observations=4, min_interval=.1, min_duration=.5, **kwargs)


def test_noise_outliers_first_bad_sample_and_unique_ids():
    e = estimator()
    e.add(12, [50, 50, 0], 1.0)
    for index in range(8):
        e.add(12, [1 + .003 * (index % 3), 2, .3], 1.3 + index * .2)
        e.add(13, [-2, 1, .2], 1.3 + index * .2)
    found = e.confirmed()
    assert set(found) == {12, 13}
    assert found[12][0] == pytest.approx([1.003, 2, .3], abs=.003)
    assert found[12][1] == 8
    e.add(12, [100, 100, 0], 4.0)
    assert e.confirmed()[12][0] == pytest.approx(found[12][0])


def test_duplicates_out_of_order_and_minimum_duration():
    e = estimator()
    for _ in range(20):
        e.add(1, [1, 2, 3], 1.0)
    e.add(1, [1, 2, 3], .5)
    assert not e.confirmed()
    for stamp in [1.11, 1.22, 1.33]:
        e.add(1, [1, 2, 3], stamp)
    assert not e.confirmed()
    e.add(1, [1, 2, 3], 1.6)
    assert set(e.confirmed()) == {1}


def test_disagreement_not_confirmed():
    e = estimator()
    for index in range(15):
        e.add(1, [index, 0, 0], index * .2)
    assert not e.confirmed()


def test_old_samples_do_not_confirm_and_reset_clears_confirmed():
    e = estimator(max_sample_age=.6)
    for index in range(3):
        e.add(1, [1, 2, 3], index * .2 + 1)
    e.add(1, [1, 2, 3], 10.0)
    assert not e.confirmed()
    for stamp in [10.2, 10.4, 10.6]:
        e.add(1, [1, 2, 3], stamp)
    assert 1 in e.confirmed()
    e.reset()
    assert not e.confirmed()


def test_confirmed_static_id_does_not_jump():
    e = estimator(window_size=5)
    for index in range(5):
        e.add(1, [1, 2, 3], index * .2 + 1)
    for index in range(10):
        e.add(1, [10, 20, 30], index * .2 + 3)
    assert e.confirmed()[1][0] == pytest.approx([1, 2, 3])


def test_nan_and_spread_rejected():
    e = estimator(inlier_radius=1.0, max_spread=.01)
    assert not e.add(1, [np.nan, 0, 0], 1)
    for index in range(6):
        e.add(1, [.1 * (index % 2), 0, 0], index * .2 + 1)
    assert not e.confirmed()
