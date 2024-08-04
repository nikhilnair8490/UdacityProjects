def check_output(output):
    """
    checking the shape of the output
    """
    if output.shape == (1, 75, 75, 16):
        print('Success!')
    else:
        print('Failure')