# DSP Software Study and Analysis  
To Study DSP concepts using STM32 on NUCLEO-G474RE  

#### Initially used [STM32F429I-DISC1_FIR_FFT_wth_Print](https://github.com/VictorTagayun/STM32F429I-DISC1_CMSIS_DSP_Tutorial) & X-CUBE-DSPDEMO as basis

### First tested on [NUCLEO-G474RE_FIR_FFT_wth_Print](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/tree/main/NUCLEO-G474RE_FIR_FFT_wth_Print)
1. import by ioc and generate code first  
2. click "Use float with printf ...."  
3. add libarm_cortexM4lf_math.a in the linker settings  

2021-02-01  
1. add FFT codes  
2. captured data on excel  
3. added "to do.txt"
		
## FIR Low Pass Filter Testing

### Data are fed to the DSP/Filter  
	
	float32_t aFIR_F32_1kHz_15kHz[TEST_LENGTH_SAMPLES] =
	{
	+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
	+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
	+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
	-0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
	-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
	-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
	+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
	+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
	+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
	+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
	-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
	-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
	+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
	+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
	+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
	+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
	-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
	-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
	-0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
	+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
	+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
	-0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
	-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
	-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
	+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f,
	+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f,
	+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f,
	+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f,
	-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f,
	-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f,
	};

Then later will be converted to [Q15](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial#q15-converted-by-dsp-function-and-printed-out-by-mcudsp-and-plotted-to-excel) and [Q31](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial#q31-converted-by-dsp-function-and-printed-out-by-mcudsp-and-plotted-to-excel) by DSP functions  
		
### The following waveforms are the printed out from MCU and plotted in Excel
	
#### Original Float 32 array as shown [above](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial#data-are-fed-to-the-dspfilter)     
	
F32 Input Signal 1kHz + 15kHz  

![F32 Input Signal 1kHz + 15kHz](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/F32_1k_15k_input.png)

FFT F32 calculated using Excel  

![FFT F32 calculated using Excel](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/F32_1k_15k_input_FFT.png)

Impulse = F32 Low Pass Filter  

![Impulse = F32 Low Pass Filter](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/F32_1k_15k_input_coeff.png)

F32 Filtered Signal 1kHz  

![F32 Filtered Signal 1kHz](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/F32_1k_filtered_output.png)

#### Q15 converted by DSP function and printed out by MCU/DSP and plotted to Excel   

Q15 Input Signal 1kHz + 15kHz (converted by DSP from Float32 to Q15)  

![Q15 Input Signal 1kHz + 15kHz](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15(conv)_1k_15k_input.png)

Impulse = Q15 Low Pass Filter  

![Impulse = Q15 Low Pass Filter](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15_LP_coeff.png)

Q15 Filtered Signal 1kHz  

![Q15 Filtered Signal 1kHz](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15_1k_filtered_output.png)

FFT Q15 Filtered Signal 1kHz Calculated by Excel  

![FFT Q15 Filtered Signal 1kHz Calculated by Excel](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15_1k_filtered_output_FFT.png)

Impulse = Q15 High Pass Filter  

![Impulse = Q15 High Pass Filter](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15_HP_coeff.png)

Q15 Filtered Signal 15kHz  

![Q15 Filtered Signal 15kHz](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15_15k_filtered_output.png)

FFT Q15 Filtered Signal 15kHz Calculated by Excel 
 
![FFT Q15 Filtered Signal 15kHz Calculated by Excel](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15_15k_filtered_output_FFT.png)

#### Q31 converted by DSP function and printed out by MCU/DSP and plotted to Excel  

Q31 Input Signal 1kHz + 15kHz (converted by DSP from Float32 to Q31)  

![31 Input Signal 1kHz + 15kHz](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q31(conv)_1k_15k_input.png)

FFT Q31 Calculated by Excel  

![FFT Q31](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q31_FFT.png)

Impulse = Q31 Low Pass Filter  

![Impulse = Q31 Low Pass Filter](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q31_LP_coeff.png)

Q31 Filtered Signal 1kHz  

![Q31 Filtered Signal 1kHz](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q31_1k_filtered_output.png)

FFT Q31 Filtered Signal 1kHz Calculated by Excel  

![FFT Q31 Filtered Signal 1kHz Calculated by Excel](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15_1k_filtered_output_FFT.png) 

	
	
## FFT Testing  

### Data are fed to the DSP  
	
	const uint16_t Sine12bit[NB_SAMPLES] =
	  {
		2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
		3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909,
		599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647,
		2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
		3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909,
		599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647
	  };

### The following waveforms are the printout from MCU and plotted in Excel. But I wrongfully converted to Float. Actually, no conversion is necessary.
	
Input DAC signal  

![Input DAC signal](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Input_Signal.png)

FFT Normalized calculated by Excel  

![FFT Normalized](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/FFT_normalized.png)

FFT Output Q15 calculated by MCU/DSP  

![FFT Output Q15](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15_FFT.png)

FFT Output Q15 (Normalized) using Excel Calculation  

![FFT Output Q15 (Normalized) using Excel Calculation](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15_FFT%20(nomalized).png)

FFT Output Float32 calculated by MCU/DSP   

![FFT Output Float32](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/F32_FFT.png)

FFT Output Float32 (normalized) using Excel Calculation  

![FFT Output Float32 (normalized) using Excel Calculation](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/F32_FFT%20(nomalized).png)

FFT Output Q31 calculated by MCU/DSP    

![FFT Output Q31](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15_FFT.png)

FFT Output Q31 (normalized) using Excel Calculation  

![FFT Output Q31 (normalized) using Excel Calculation](https://github.com/VictorTagayun/NUCLEO-G474RE_CMSIS_DSP_Tutorial/blob/main/NUCLEO-G474RE_FIR_FFT_wth_Print/captured_data%26plot/Q15_FFT%20(nomalized).png)


### Other References :

[STM32F429I-DISC1_FIR_FFT_wth_Print Project](https://github.com/VictorTagayun/STM32F429I-DISC1_CMSIS_DSP_Tutorial)

[Use of FMAC for FIR Low Pass Filter](https://github.com/VictorTagayun/NUCLEO-G474RE_FMAC_Study_and_Analysis)

[Use of FMAC for **Real Time** FIR/IIR Filter](https://github.com/VictorTagayun/NUCLEO-G474RE_RealTime_FIR_IIR_FMAC)


*Disclaimer:*
[Updated Disclaimer](https://github.com/VictorTagayun/GlobalDisclaimer)

*The projects posted here are for my Personal reference, learning and educational purposes only.*
*The purpose of a certain project may be for testing a module and may be just a part of a whole project.*
*It should not be used in a production or commercial environment.*
*Any cause of injury and/or death is the sole responsibility of the user.*
