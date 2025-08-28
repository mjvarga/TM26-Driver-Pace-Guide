# TM26-Driver-Pace-Guide-Research

# Purpose 
The purpose of implementing a Driver Pace Guide System is to assist the driver in the level of aggressiveness/how much does the driver “push” the car. Based on the light, the driver would adjust how their pace is relative to the estimated power needed to complete the endurance run vs the power remaining in the accumulator. 

# Background
In Formula SAE endurance, efficiency is as important as outright lap time. Drivers have to carefully manage energy efficiency while maintaining competitive pace. Some driver pace displays may often overwhelm the driver with interpreting data or and performing “math”/calculations mid-race, which should rather be avoided by making the displays more simple and quick to understand. An intuitive system is required to maximize both vehicle and driver performance.

# Research
The University of Wisconsin FSAE team (#1 in 2025) worked on their own driver pace display that compares accumulator state of charge (SOC) with the endurance distance remaining. It also displays the distance shown as a countdown percentage from 100%, and their system uses a simple LED bar to represent the energy-to-distance ratio (theoretical representations):

- Energy (green) lights balanced → On target pace.
<img width="771" height="132" alt="image" src="https://github.com/user-attachments/assets/a8d01227-c36d-4019-ad70-fa72b68c32f4" />

- Bigger energy lights → Surplus energy; driver may push harder.
<img width="773" height="131" alt="image" src="https://github.com/user-attachments/assets/5bcc18c1-18ba-4e76-82e2-a80747a597c6" />

- Lower energy lights → Energy deficit; driver must slow down.
<img width="774" height="132" alt="image" src="https://github.com/user-attachments/assets/7533e2a4-ab4f-4a8a-a508-6f2d159e5024" />

This design should eliminate “mental math” for the driver, providing instant/glanceable information on how to drive. Note: Their system also integrates switchable traction control and torque vectoring modes.

# Design
- Driver Pace Guide: LED strip tied to SOC vs. distance ratio, with a neutral “on pace” midpoint.
- Mode Indication: Dash display for traction control and torque vectoring states, backed up with direct ECU readout.
- Driver Interface: Information is reduced to simple, glanceable cues. No calculations or detailed interpretation required during driving → Intuition is key.
<img width="733" height="716" alt="image" src="https://github.com/user-attachments/assets/0eb15eb3-d54e-423a-aac3-ccbbde005cd9" />

# Rough Notes (Michael & Owais)
- What the Wisconsin FSAE team has on their dashboard is the state of charge counting down from 100%
   
- They also have the distance starting at the beginning of endurance counting down from 100% which the driver can see from the dash to see if the energy is bigger than the distance remaining (indicates you can go faster) and if the energy is smaller than the distance remaining (indicates you should slow down to save energy as you are driving the car.)
  
- What the judge particularly likes about the design is that they have used lights across the top of the dash as an indicator for the relationship between those two values.
  
- The description of the lights is a “simple ratio between those two” where the driver “is not doing any math”
  
- The driver at the side of their eye can see the amount of lights that are lit
  
- If the lights are right in the middle → You are on pace (balance between speed and energy usage)
  
- If lights go up → You got more energy (Go a little bit faster)
  
- If lights go down → You have less energy (Slow it down so it goes back to the middle)
  
- Key tip: Don’t make your drivers do math
  
- They have switchable torque vectoring and switchable traction control
  
- "Whenever the driver throws that switch it flashes up on the dash really big and to see what number they landed on really big"
  
- Also they have a redundancy where if that system is trying to make the number that flashes up (if it fails) the driver still has at the bottom of the corner a readout directly from the ECU telling them which mode they're in.
  
- "They have two simple systems on the dash for both of those pieces of information to help the driver get the most out of the car"

Video Resource of Design Judges: https://www.youtube.com/watch?v=SJEev6Q0B40 (start at 1:08:00)




