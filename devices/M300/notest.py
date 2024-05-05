import time 

log1= 'normal excecuting'
log2= 'warning  print'
log3= 'error in exceecuting'
log4= 'Error raretly'

if __name__ == '__main__':
    count = 0
    while True:
        time.sleep(1)
        count += 1
        print(log1  )
        if count % 5 == 0:
            print(log2)
        if count % 10 == 0:
            print(log3) 
        if count % 15 == 0: 
            print(log4)
            