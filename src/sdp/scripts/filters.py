last_diff = None
def maxChangeFilter(in_val, maxDiff=10):
    global last_diff
    if last_diff:
        diff = in_val - last_diff 
        if diff >= maxDiff:
            v = last_diff + maxDiff
            last_diff = v
            return v
        if diff <= -maxDiff:
            v = last_diff - maxDiff
            last_diff = v
            return v
        else:
            last_diff = in_val
            return in_val
    else:
        last_diff = in_val
        return in_val
