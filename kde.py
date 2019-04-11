import numpy as np
from fastkde import fastKDE
from matplotlib import pyplot as plt
from scipy.stats import gaussian_kde

width = 300
height = 100





class realtime_kde:

    def __init__(self, width, height):
        self.width = width
        self.height = height

        # setup coordinates grid for this kde
        self.x, self.y = np.mgrid[0:width+1,0:height+1]
        self.coor_grid = (self.x.ravel(),self.y.ravel())
        self.discret_coordinate = np.array(list(zip(*self.coor_grid)))

        # setup plot
        plt.ion()
        _fig, self.ax = plt.subplots(1,1)

        # inverse y axis
        axes = plt.gca()
        axes.set_xlim([0,width])
        axes.set_ylim([0,height])
        axes.invert_yaxis()

        # dummpy data at init
        empty_points = np.array([0,0])
        empty_points = np.random.rand(2,5) * 10
        kde = gaussian_kde(empty_points)
        # setup plot
        z = kde(self.coor_grid).reshape(*self.x.shape)
        pc = self.ax.pcolor(self.x,self.y,z)
        self.cb = plt.colorbar(pc)
        self.cb.ax.set_ylabel('Probability density')

        self.prob = None

        plt.show()

    def update_kde(self, points):
        myPDF, axes = fastKDE.pdf(points[0],points[1])
        # print(' pdf sum: ',myPDF.sum())
        _a = myPDF
        _b = axes
        # print(myPDF.shape)
        # print(np.array(axes).shape)
        # exit()
        # myPDF /= myPDF.sum()

        x,y = axes
        pc = self.ax.pcolor(x,y, myPDF)
        self.cb.on_mappable_changed(pc)

        return _a, _b

        # kernel density estimate of the PDF
        # print(1)
        # kde = gaussian_kde(points)
        # print(2)
        # # print(dir(kde))
        # # exit()
        # # evaluate the estimated PDF on a grid
        #
        # if self.prob is None:
        #     self.prob = kde(self.coor_grid).reshape(*self.x.shape)
        # else:
        #     self.prob += kde(self.coor_grid).reshape(*self.x.shape)

        print(3)
        # At this point prob is the probability of having an obstacle
        # We want the probability of having free space so we inverse the probability
        # First multiple the probability by the percision of digit we want so we
        # wont loose percision
        # print(self.prob)
        ########################################################################
        # self.prob = 1 - self.prob

        ########################################################################
        self.prob /= self.prob.sum()
        print(4)
        # # print(self.prob)
        # self.prob = 1 - self.prob
        # self.prob /= self.prob.sum()
        # ########################################################
        # self.prob = 1 - self.prob
        # self.prob /= self.prob.sum()
        ########################################################################
        # percision = 4
        # self.prob * 10**percision
        # self.prob = 10**(percision+2)-self.prob
        # # make probability sums to one
        # self.prob /= self.prob.sum()
        #
        # if 1:
        #     percision = 4
        #     self.prob * 10**percision
        #     self.prob = 10**(percision+2)-self.prob
        #     # make probability sums to one
        #     self.prob /= self.prob.sum()

        pc = self.ax.pcolor(self.x, self.y, self.prob)
        self.cb.on_mappable_changed(pc)
        print(5)

    def get_coor(self):
        """ Return a coordinate according to kde probability"""
        print('sums to {}'.format(self.prob.sum()))
        # print(self.discret_coordinate)
        # print(self.discret_coordinate.shape)

        # exit()
        # return None05
        # return None
        # print(self.prob)
        # return None
        idx = np.random.choice(len(self.discret_coordinate), p=self.prob.ravel())
        return self.discret_coordinate[idx]
        # return np.random.choice(self.discret_coordinate, p=self.prob.ravel())

#
#
#

if __name__ == "__main__":

    kde = realtime_kde(width, height)

    while True:
        points = np.random.rand(2,3)
        # print(points)
        ########################################
        # points = points * 0.1 + 0.5
        ########################################
        # points = [[0.5] * 20]
        # points.append(points)
        # points = np.array([points[0], points[1]])
        # print(points)
        # exit()
        points[0] = points[0] * width
        points[1] = points[1] * height

        print('>')
        kde.update_kde(points)
        print(kde.get_coor())
        print('<')
        plt.pause(0.05)
        print('<')
