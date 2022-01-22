# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?

2. What is the difference between a class and a struct in C++?
Ans : The member variables and methods in a struct are public by default whereas in a struct they are private by default.


3. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specific C++ core guidelines in your answer)?
Ans: 
a) A class is defined for invariant data, meaning that the members of the class are related to each other and therefore cannot be independent of each. For instance, a class will be defined for creating an object and the object will hold data all related to that entity. Variables in a structure can be independent of each other. 
b) To maintain consistency for accessing private data, it is better to use a class if all the member variables are to be set private which helps with readability and makes code less complicated.

References : 1) C.8: Use class rather than struct if any member is non-public
             2) C.2: Use class if the class has an invariant; use struct if the data members can vary independently


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
Ans: explicit keyword helps to avoid conversions which are not intended by user. For instance if a function is defined a string return type
and the function is actually supposed to return an int, this would unnecessary conversion. Adding explicit keyword helps avoid that.

Reference : C.46: By default, declare single-argument constructors explicit



5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
Ans: Declaring Transform2D::inv() as const makes sure that the member function does not change the value of the member variables. This can help avoid making mistakes and makes the code more efficient.

Reference : Con.2: By default, make member functions const
