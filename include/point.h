class point
{
public:
    float lat;
    float lon;
    point(float x, float y);
    ~point();
};

point::point(float x, float y)
{
    this->lat = x;
    this->lon = y;
}

point::~point()
{
}
