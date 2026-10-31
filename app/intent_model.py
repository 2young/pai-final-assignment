"""Small Japanese text-intent classifier. No remote LLM or GPU required."""

from pathlib import Path
import joblib
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.pipeline import make_pipeline
from sklearn.svm import LinearSVC

MODEL_PATH = Path(__file__).parent / "intent_model.joblib"

EXAMPLES = {
    "FORWARD": [
        "前に進んで", "前進して", "まっすぐ進んで", "前へ移動して", "進め",
        "前に移動", "少し前進", "前方向に走って", "そのまま直進", "真っ直ぐ進もう",
        "前に行こう", "前へ進む", "前方に進んで", "もう少し進んで", "直進してください",
    ],
    "BACKWARD": [
        "後ろに下がって", "後退して", "バックして", "後ろへ進んで", "戻って",
        "少し後退", "後ろ方向に移動", "後退してください", "後ろに進む", "後ろに移動",
        "後ろへ下がって", "ゆっくりバック", "バックしてください", "少し下がって", "後方へ進んで",
    ],
    "LEFT": [
        "左に曲がって", "左を向いて", "左に旋回", "左方向に回って", "左回りに向きを変えて",
        "左へ曲がって", "左へ向いて", "左方向へ旋回して", "左に回って", "左へ回転",
        "反時計回りに回って", "左向きにしてください", "左を向く", "左に向きを変えて", "左旋回",
    ],
    "RIGHT": [
        "右に曲がって", "右を向いて", "右に旋回", "右方向に回って", "右回りに向きを変えて",
        "右へ曲がって", "右へ向いて", "右方向へ旋回して", "右に回って", "右へ回転",
        "時計回りに回って", "右向きにしてください", "右を向く", "右に向きを変えて", "右旋回",
    ],
    "STOP": [
        "止まって", "停止して", "ストップ", "そこで停止", "動かないで",
        "停車して", "ここで止まれ", "止めて", "そこで止まって", "動きを止めて",
        "完全停止", "その場で停止", "停止してください", "ストップして", "一時停止して",
    ],
}


def train(path=MODEL_PATH):
    texts = []
    labels = []
    for label, examples in EXAMPLES.items():
        texts.extend(examples)
        labels.extend([label] * len(examples))
    clf = make_pipeline(
        TfidfVectorizer(analyzer="char", ngram_range=(1, 4)),
        LinearSVC(random_state=42),
    )
    clf.fit(texts, labels)
    joblib.dump(clf, path)
    return clf


def predict(clf, text):
    # Fail-safe for obviously unrelated commands: choose not to drive.
    if not text.strip():
        return "STOP"
    vec = clf.named_steps["tfidfvectorizer"].transform([text])
    if vec.nnz == 0:
        return "STOP"
    return str(clf.predict([text])[0])
