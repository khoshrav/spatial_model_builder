from model import compile_model
from helpers import early_stopping, model_checkpoint, tensor_board, get_data

if __name__ == '__main__':
    model = compile_model()
    callbacks = [early_stopping(), model_checkpoint(), tensor_board()]
    train_batch, val_batch = get_data()
    model.fit_generator(generator=train_batch, steps_per_epoch= len(train_batch),
                        validation_data=val_batch, validation_steps=len(val_batch),
                        callbacks=callbacks, epochs=100, verbose=0, max_queue_size=3)
