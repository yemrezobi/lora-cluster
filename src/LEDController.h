class LEDController{
    public:
        LEDController(int pin);
        LEDController(int pin, unsigned long periods[], int length, bool repeat = false);
        ~LEDController();
        void setPeriods(unsigned long periods[], int length, bool repeat = false);
        bool poll();

    private:
        int _pin;
        unsigned long* _periods;
        int _arrayLen;
        int _counter;
        unsigned long _startTime;
        bool _pinState;
        bool _repeat;
};