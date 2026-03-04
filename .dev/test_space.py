from skillet import ParameterizedDiscrete

if __name__ == "__main__":
    space = ParameterizedDiscrete(n="n_options", start=0)
    bound = space.bind(n_options=10)
    print(bound)
    for i in range(10):
        print(bound.sample())