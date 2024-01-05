#!/usr/bin/python

import argparse
import sys

class Py514(object):
    def __init__(self, xtal_corr = 0.0):
        ''' 
        Initialization routine for the py514 class.

        xtal_corr = xtal correction factor such that actual
                    xtal frequency = XTAL_FREQ (1.0 + xtal_corr / 10000000.0)
                    You can think of it as the xtal error in Hz
                    when the xtal frequency is normalized to 
                    10 MHz.
        '''
        # Define needed constants.
        self.__xtal_freq = float(31980000.0)

        # Initialize properties.
        self.__xtal_corr = float(xtal_corr);

        # Initialize class variables.
        self.__m_int = -1
        self.__m_frac = -1
        self.__freq = 0.0

        # Constant definitions
        self.__REG_LP = 0
        self.__REG_M_FRAC1 = 5
        self.__REG_M_FRAC2 = 6
        self.__REG_M_FRAC3 = 7
        self.__REG_M_INT_FRAC = 8
        self.__REG_M_INT = 9
        self.__REG_HS_DIV = 10
        self.__REG_LS_HS_DIV = 11
        self.__REG_OE_STATE = 14
        self.__REG_RESET = 128
        self.__REG_CONTROL = 132

        self.__MIN_FREQ = 100000
        self.__MAX_FREQ = 250000000

        self.__FXO = 31980000
        self.__FVCO_MIN = 2080000000
        self.__FVCO_MAX = 2500000000

        self.__HS_DIV_MIN = 10
        self.__HS_DIV_MAX = 1022

        self.__OUTPUT_ENABLE = 0x04
        self.__RESET = 0x80
        self.__CALIBRATE = 0x01

        return


    # Properties
    #
    @property
    def xtal_corr(self):
        ''' 
        Property encapsulating the crystal correction factor.

        '''
        return self.__xtal_corr
      
    @xtal_corr.setter
    def xtal_corr(self, xtal_corr):
        ''' 
        Setter encapsulating the crystal correction factor.

        '''
        self.__xtal_corr = float(xtal_corr)
        return
            

    # Clock routines
    #
    def corrected_xtal_freq(self):
        ''' 
        Calculate a xtal frequency that takes the
        correction factor into account.

        '''
        freq = float(self.__FXO) * (1.0 + self.xtal_corr / 10000000.0)
        return int(freq)

    def set_freq_large(self, freq = 10000000):
        ''' 
        Program a large frequency change.

        '''
        # Disable the output
        #
        #self.output_disable()
        
        # Calculate rqeuired constants
        #
        ls_div = self.calculate_ls_div(freq)
        hs_div = self.calculate_hs_div(freq, ls_div)
        xtal_freq = self.corrected_xtal_freq()
        m_int, m_frac = self.calculate_m(ls_div, hs_div, freq, xtal_freq)
        lp1, lp2 = self.calculate_lp(m_int, m_frac)

        print('m_int: {}'.format(m_int))
        print('m_frac: {}'.format(m_frac))
        print('hs_div: {}'.format(hs_div))
        print('ls_div: {}'.format(ls_div))
        print('lp1: {}'.format(lp1))
        print('lp2: {}'.format(lp2))

        # Program the I2C registers
        #
        reg00 = (int(lp1) << 4) + (int(lp2) << 0)
        reg05 = (int(m_frac) >>  0) & 0x00FF
        reg06 = (int(m_frac) >>  8) & 0x00FF
        reg07 = (int(m_frac) >> 16) & 0x00FF
        reg08 = (int(m_int)  >>  0) & 0x0007
        reg08 = (reg08 << 5) + ((int(m_frac) >> 24) & 0x001F)
        reg09 = (int(m_int)  >> 3) & 0x003F
        reg10 = (int(hs_div) >> 0) & 0x00FF
        reg11 = (int(ls_div) >> 0) & 0x0007
        reg11 = (reg11 << 4) + ((int(hs_div) >> 8) & 0x0003)

        #self.write_to_register(self.__REG_LP, reg00)
        #self.write_to_register(self.__REG_M_FRAC1, reg05)
        #self.write_to_register(self.__REG_M_FRAC2, reg06)
        #self.write_to_register(self.__REG_M_FRAC3, reg07)
        #self.write_to_register(self.__REG_M_INT_FRAC, reg08)
        #self.write_to_register(self.__REG_M_INT, reg09)
        #self.write_to_register(self.__REG_HS_DIV, reg10)
        #self.write_to_register(self.__REG_LS_HS_DIV, reg11)
        print('{} (0x{:x}): 0x{:x}'.format('REG_LP', self.__REG_LP, reg00))
        print('{} (0x{:x}): 0x{:x}'.format('REG_M_FRAC1', self.__REG_M_FRAC1, reg05))
        print('{} (0x{:x}): 0x{:x}'.format('REG_M_FRAC2', self.__REG_M_FRAC2, reg06))
        print('{} (0x{:x}): 0x{:x}'.format('REG_M_FRAC3', self.__REG_M_FRAC3, reg07))
        print('{} (0x{:x}): 0x{:x}'.format('REG_M_INT_FRAC', self.__REG_M_INT_FRAC, reg08))
        print('{} (0x{:x}): 0x{:x}'.format('REG_M_INT', self.__REG_M_INT, reg09))
        print('{} (0x{:x}): 0x{:x}'.format('REG_M_HS_DIV', self.__REG_HS_DIV, reg10))
        print('{} (0x{:x}): 0x{:x}'.format('REG_LS_HS_DIV', self.__REG_LS_HS_DIV, reg11))

        # Calibrate the VCO
        #
        #self.calibrate_vcxo()

        # Re-enable the output
        #
        #self.output_enable()

        # Store values for later use.
        #
        self.__m_int = m_int
        self.__m_frac = m_frac
        self.__freq = freq
        return
    
#    def set_freq_small(self, freq):
#        ''' 
#        Program a small frequency change.
#
#        '''
#        # Get the current value of m
#        #
#        reg05 = self.read_from_register(self.__REG_M_FRAC1)
#        reg06 = self.read_from_register(self.__REG_M_FRAC2)
#        reg07 = self.read_from_register(self.__REG_M_FRAC3)
#        reg08 = self.read_from_register(self.__REG_M_INT_FRAC)
#        reg09 = self.read_from_register(self.__REG_M_INT)
#        
#        m_frac = ((reg08 & 0x1F) << 24) + (reg07 << 16) + (reg06 << 8) + (reg05 << 0)
#        m_int  = ((reg08 >> 5) & 0x07) + ((reg09 & 0x1F) << 3)
#        m_old = float(m_int) + float(m_frac) / float(1 << 29)
#
#        # Check to see if the values read from the chip equal
#        # the expected values.  If they do calculate new m values
#        # and write them out.  If not raise an exception.
#        #
#        if ((m_frac == self.__m_frac) and (m_int == self.__m_int)):            
#            # Values match.  Calculate the new m value.
#            #
#            m_new = m_old * float(freq) / float(self.__freq)
#
#            # Write the new values to the clock.
#            #
#            reg05 = (int(m_frac) >>  0) & 0x00FF
#            reg06 = (int(m_frac) >>  8) & 0x00FF
#            reg07 = (int(m_frac) >> 16) & 0x00FF
#            reg08 = (int(m_int)  >>  0) & 0x0007
#            reg08 = (reg08 << 5) + ((int(m_int) >> 24) & 0x001F)
#            reg09 = (int(m_int)  >> 3) & 0x003F  
#
#            self.write_to_register(self.__REG_M_FRAC1, reg05)
#            self.write_to_register(self.__REG_M_FRAC2, reg06)
#            self.write_to_register(self.__REG_M_FRAC3, reg07)
#            self.write_to_register(self.__REG_M_INT_FRAC, reg08)
#            self.write_to_register(self.__REG_M_INT, reg09)    
#        else:
#            # Values don't match.  Raise the exception.
#            #
#            raise ValueError(\
#                "Expected m value of ("+ str(self.__m_int) + ", " + str(self.__m_frac) + ") " + \
#                "does not match value of (" + str(m_int) + ", " + str(m_frac) + ") read from clock")
#
#        return      


    # Support routines.
    #
    def calculate_ls_div(self, freq):
        ''' 
        Calculate the value of ls_div based on frequency.

        '''
        # The minimum value of ls_div occurs when the vcxo is
        # at its minimum and hs_div is at its maximum.
        #
        hsdiv_freq = self.__HS_DIV_MAX * freq
        ls_div = 0

        if (hsdiv_freq < self.__FVCO_MIN):
            hsdiv_freq = hsdiv_freq * 2
            ls_div = 1
        if (hsdiv_freq < self.__FVCO_MIN):
            hsdiv_freq = hsdiv_freq * 2
            ls_div = 2
        if (hsdiv_freq < self.__FVCO_MIN):
            hsdiv_freq = hsdiv_freq * 2
            ls_div = 3                    
        if (hsdiv_freq < self.__FVCO_MIN):
            hsdiv_freq = hsdiv_freq * 2
            ls_div = 4
        if (hsdiv_freq < self.__FVCO_MIN):
            hsdiv_freq = hsdiv_freq * 2
            ls_div = 5
        return ls_div

    def calculate_hs_div(self, freq, ls):
        ''' 
        Do a binary search to find the minimum value of hs_div
        that will result in a vco frequency in the allowed range.

        '''
        ls_div = (1 << ls)
        lsdiv_freq = ls_div * freq
        hs_min = self.__HS_DIV_MIN / 2
        hs_max = self.__HS_DIV_MAX / 2
        fvco = self.__FVCO_MIN / 2

        for i in range(0, 10):
            hs_div = int((hs_min + hs_max) / 2)
            if (lsdiv_freq * hs_div >= fvco):
                hs_max = hs_div
            else:
                hs_min = hs_div
        return (2 * hs_max)

    def calculate_m(self, ls_div, hs_div, freq, xtal_freq):
        ''' 
        Calculate the value of m to give the desired frequency

        '''
        m_t = float(1 << ls_div) * float(hs_div) * float(freq) / float(xtal_freq)
        m_int = int(m_t)
        m_frac = int(((m_t - float(m_int)) * (1 << 29))+0.5);
        return (m_int, m_frac)

    def calculate_lp(self, m_int, m_frac):
        ''' 
        Calculate the values of LP1 and LP2 based on the 
        value of m

        '''
        m = float(m_int) + float(m_frac) / float(1 << 29)
        if (m < 65.259980246):
            lp1 = 2
            lp2 = 2
        elif (m < 67.859763463):
            lp1 = 2
            lp2 = 3
        elif (m < 72.937624981):
            lp1 = 3
            lp2 = 3
        elif (m < 75.843265046):
            lp1 = 3
            lp2 = 4
        else:
            lp1 = 4
            lp2 = 4
        return (lp1, lp2)


                    
if __name__ == "__main__":
    ''' 
    Main routine to set the frequency on an Si514 clock.
    
    '''
    # Parse the command line arguments.
    #
    parser = argparse.ArgumentParser(description='Set the frequency of an Si514 clock')
    parser.add_argument('-f', dest='freq', nargs=1, type=int, help='frequency in Hz')
    args = parser.parse_args()

    # Only a single frequency argument.  If it's not given go to a 
    # default value.  If the value is out of range print an error
    # and exit.
    #
    freq = 10000000
    freq_min = 100000
    freq_max = 170000000

    if (not(args.freq == None)):
        freq = args.freq[0]
        sys.stdout.write("Setting clock to frequency of " + str(freq) + "\r\n")
    else:
        sys.stdout.write("Using default frequency of " + str(freq) + "\r\n")

    if (freq < freq_min) or (freq > freq_max):
        sys.stderr.write("Error: Frequency must be in the range from " + \
                          str(freq_min) + " " + \
                         "Hz to " + \
                          str(freq_max) + " " + \
                         "Hz\n\r")
        sys.exit()

    py514 = Py514()
    py514.set_freq_large(freq)
