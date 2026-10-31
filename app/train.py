from intent_model import MODEL_PATH, train

if __name__ == "__main__":
    train()
    print(f"Trained model saved to {MODEL_PATH}")
