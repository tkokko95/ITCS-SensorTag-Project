float calc_mean(float data[80][3], int data_col)
{
    int i;
    float sum = 0.0, mean;
    for(i = 0; i < 80; i++)
    {
        sum += data[i][data_col];
    }
    mean = sum / 80;
    return mean;
    
}

int find_peaks(float data[80][3], int col, float mean)
{
    float threshold = 0.25; //tiedän, tämä vaatinee säätöä
    char i = 0, peaks = 0;
    bool bypass = false;
    while(i < 80)
    {
        if((abs(mean - data[i][col]) >= threshold && !bypass))
        {
            peaks++;
            i++;
            bypass = true;
        }
        
        if((abs(mean - data[i][col]) < threshold && bypass))
        {
            i++;
            bypass = false;
        }
        
        else
        {
            i++;
        }
        
    }
    return peaks;
}
