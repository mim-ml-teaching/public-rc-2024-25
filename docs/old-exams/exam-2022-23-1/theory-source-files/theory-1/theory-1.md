# Robot Control Exam, Task 1 -- Linearization around Fixed Points

Recall from the classes that we can model population growth using the so called logistic model:

$$\dot{x} = x(P_{max}-x)$$

where \\(x\\) is the population size and \\(P_{max}\\) is the population limit above which the environment becomes resource scarse.

Now consider the more complicated situation when two species are competing with each other.
In this situation the change in population depends also on the amount of interactions between two populations:

$$\dot{x} = x(P_{max}-x) - a \cdot g(x,y)$$

where \\(y\\) is the current population size of the second species, \\(a\\) is some parameter and \\(g\\) is some function of \\(x\\) and \\(y\\) describing the interactions between species.

Your task is to use the above model for a competition between rabits and wolves - population sizes denoted by \\(r\\) and \\(w\\) respectively.
You have to:

1. (20%) write the ODEs describing the system assuming that:
    - the number of interactions between two populations is simply the number of (rabit, wolf) pairs that can be created in the environment
    - \\(P_{max}\\) is equal to 4 for rabits and 3 for wolves
    - \\(a\\) is equal to 2 for rabits and 1 for wolves
    - your state is \\\vec{x} = [r, w]^T\\)
2. (20%) find fixed points of the created system
3. (60%) linearize dynamics of the system around those fixed points
