import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
from intent_model import predict, train


def test_predictions(tmp_path):
    model = train(tmp_path / 'model.joblib')
    checks = {
        '前へ移動してください': 'FORWARD',
        '右側に向きを変えて': 'RIGHT',
        'もう少し前に行って': 'FORWARD',
        '左に向いてください': 'LEFT',
        'そのまま前進して': 'FORWARD',
        'ここで止めて': 'STOP',
        '少し後ろへ移動して': 'BACKWARD',
        '': 'STOP',
        '赤いオブジェクトまで行ってください': 'GO_TO_OBJECT',
        '赤い箱まで行って': 'GO_TO_OBJECT',
        '紫の箱まで': 'GO_TO_OBJECT',
        '赤い箱に向かわず停止して': 'STOP',
    }
    for sentence, expected in checks.items():
        assert predict(model, sentence) == expected, sentence
