import numpy as np

from cube_vision import trash_eval as T


def make(rng, n_classes=4, per_class=6, noise=0.25, domain_shift=0.0, dim=32):
    centres = rng.normal(size=(n_classes, dim))
    shift = rng.normal(size=dim) * domain_shift
    def sample(c):
        v = centres[c] + rng.normal(size=dim) * noise + shift
        return v / np.linalg.norm(v)
    labels = [f"c{c}" for c in range(n_classes) for _ in range(per_class)]
    base = np.array([sample(c) for c in range(n_classes) for _ in range(per_class)])
    return centres, base, labels, sample


def test_rank_picks_the_nearest_class_and_reports_the_margin():
    rng = np.random.default_rng(0)
    centres, base, labels, sample = make(rng)
    q = sample(2)
    label, score, margin = T.rank(np.stack([q, q, q, q]), base, labels)
    assert label == "c2" and score > 0.5 and margin > 0.1


def test_real_camera_samples_fix_a_domain_shift_and_leave_one_out_is_honest():
    rng = np.random.default_rng(1)
    # base DB = clean images; the camera sees a shifted domain
    centres, base, labels, _ = make(rng, noise=0.2)
    shift = rng.normal(size=32) * 1.2
    real, real_labels = [], []
    for c in range(4):
        for _ in range(5):
            v = centres[c] + rng.normal(size=32) * 0.2 + shift
            v /= np.linalg.norm(v)
            real.append(np.stack([v] * 4))
            real_labels.append(f"c{c}")
    result = T.evaluate(np.array(real), real_labels, base, labels)
    assert result["base_plus_real"]["accuracy"] >= result["base_only"]["accuracy"]
    assert result["base_plus_real"]["accuracy"] >= 0.8
    assert result["base_plus_real"]["n"] == 20
    # leave-one-out: a lone real sample can never be matched against itself
    one = T.evaluate(np.array(real[:1]), real_labels[:1], base, labels)
    assert one["base_plus_real"]["n"] == 1


def test_threshold_recommendation_keeps_precision():
    rows = [("a", "a", 0.9, 0.3)] * 10 + [("a", "b", 0.45, 0.01)] * 3 + [("b", "b", 0.7, 0.2)] * 5
    summary = T.summarise(rows)
    assert summary["threshold_for_97pct_precision"] > 0.45
    assert summary["accuracy"] == 15 / 18 and summary["top_confusions"][0] == (("a", "b"), 3)
