from machine import Pin, ADC

def get_PotADC1() -> float:
    PotADC1 = ADC(Pin(26))
    MAX_ADC_VALUE = 65536
    value = PotADC1.read_u16() / MAX_ADC_VALUE
    return value
    
def get_PotADC2() -> float:
    PotADC2 = ADC(Pin(27))
    MAX_ADC_VALUE = 65536
    value = PotADC2.read_u16() / MAX_ADC_VALUE
    return value
