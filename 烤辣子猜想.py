# The Collatz Sequence --考拉兹猜想

# Using While loop
while True:
#     Number Enter Part
    number = input('Input your number here: ')
#     Print 'quit' to stop program
    if number == 'quit':
        break
#     Check Input Part    
    try:
        number = int(number)
#         Analysis Part
        while number != 1:
#         even number
            if number % 2 == 0:
                number = number/2
                print(number)
                continue
                     
#         odd number    
            if number % 2 == 1:
                number = number * 3 + 1
                print(number)
                continue

    except ValueError:
        print('Please enter valid integer: ')