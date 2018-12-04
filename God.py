class God:
    # TODO design omni-potent class
    # controls each car
    # interface to periphery
    # r/w from/to txt file (or sth else)
    # use (instance of CarFree2D).set_destination(x,y) to push data to each car
    # maybe: create a log file (CSV Data to copy it to excel etc to show graphs of velocity, acceleration, etc)...
    # ...for presentation
    #current_dir = os.path.dirname(os.path.abspath(__file__))
    #file_path = os.path.join(current_dir, "path.txt")
    #path_file = pygame.image.load(image_path)
    def file_read(self):
        fileToBeRead = open("path.txt", "r")
        xCoord = 0.0
        yCoord = 0.0
        timestamp = 0 #milliseconds
        destination = False
        for line in fileToBeRead:
            xCoord = line.split(',')[0]
            yCoord = line.split(',')[1]
            timestamp = line.split(',')[2]
            destination = line.split(',')[3]
            #print(xCoord,yCoord,timestamp,destination)
            car1 = CarFree2D(0, 0)
            car1.set_destination(10, 10, time.time() + 10)