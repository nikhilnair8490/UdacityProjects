import tensorflow as tf
import solution.utils

def softmax(logits):
    """
    softmax implementation
    args:
    - logits [tensor]: 1xN logits tensor
    returns:
    - soft_logits [tensor]: softmax of logits
    """
    # IMPLEMENT THIS FUNCTION
    
    soft_logits = tf.nn.softmax(logits)
    
    return soft_logits


def cross_entropy(scaled_logits, one_hot):
    """
    Cross entropy loss implementation
    args:
    - scaled_logits [tensor]: NxC tensor where N batch size / C number of classes
    - one_hot [tensor]: one hot tensor
    returns:
    - loss [tensor]: cross entropy 
    """
    # IMPLEMENT THIS FUNCTION
    
    soft_logits = softmax(scaled_logits)
    
    # Calculate categorical cross-entropy loss (assuming multi-class classification)
    nll = tf.keras.losses.CategoricalCrossentropy(from_logits=True)(one_hot, soft_logits)

    return nll


def model(X, W, b):
    """
    logistic regression model
    args:
    - X [tensor]: input HxWx3
    - W [tensor]: weights
    - b [tensor]: bias
    returns:
    - output [tensor]
    """
    # IMPLEMENT THIS FUNCTION
    
    # logistics regression model
    output = tf.matmul(X, W) + b
    
    return output


def accuracy(y_hat, Y):
    """
    calculate accuracy
    args:
    - y_hat [tensor]: NxC tensor of models predictions
    - y [tensor]: N tensor of ground truth classes
    returns:
    - acc [tensor]: accuracy
    """
    # IMPLEMENT THIS FUNCTION
    
    # Calculate accuracy
    acc = tf.reduce_mean(tf.cast(tf.equal(tf.argmax(y_hat, axis=1), tf.argmax(Y, axis=1)), tf.float32))
    
    return acc


if __name__ == "__main__":
    # pass
    # Add any code to test your implementation here
    # For example, you may want to test your implementation with the following data
    logits = tf.constant([[1.0, 2.0, 3.0]])
    one_hot = tf.constant([[0.0, 0.0, 1.0]])
    print(cross_entropy(logits, one_hot))
    print(softmax(logits))
    utils.check_softmax(softmax)
    #utils.check_ce(cross_entropy)
    utils.check_model(model)
    utils.check_acc(accuracy)