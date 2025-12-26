[h1]How to Use TrinityOS[/h1]
----------------------------------------

[h2]Step 1 – Build the System[/h2]

Build the setup as shown in the screenshots.

You will need:
• Two Satellite Dishes  
  • IMPORTANT: Build both dishes facing NORTH on the player compass.
    If you don’t, you must edit one of the chips.  
    <INSERT GUIDE FOR EDITS HERE>  
• 3 StructureLogicButtons  
• 1 LogicDial  
• 1 Large LED Display  
• 1 Computer: IC Editor Chip + Comms Motherboard  
• 9 IC10 Housings  
• 9 IC10 Chips  
• Everything on ONE heavy cable network, with NOTHING else connected

You may use modular consoles as long as you update device names in the scripts.

----------------------------------------

[h2]Step 2 – Name the Chip Housings & Buttons[/h2]

Each IC Housing must be named exactly after the script it runs.

Chip Housing Names:
• TrinityOS  
• TrinitySearch  
• TrinityAngleSolver  
• TrinityPolynomial  
• TrinityPolynomial2  
• TrinityOne  
• TrinityTwo  
• TrinityThree  
• TrinityFour  

The Stack Loader scripts are NOT separate chips.  
They run on:
• TrinityPolynomial  
• TrinityPolynomial2  
• TrinityOS

Button Names:
• Yes (green)  
• No (red)  
• Enter (grey)

----------------------------------------

[h2]Step 3 – Load & Run the Stack Loaders[/h2]

1. Load PolynomialStackLoader onto BOTH polynomial chips.  
   Run them both.  

2. Load TrinityOSStackLoader onto the TrinityOS chip.  
   Run it.

3. Once all three finish loading, TURN THEM OFF.

----------------------------------------

[h2]Step 4 – Load the Main Scripts[/h2]

Load each script into its matching housing name:
• TrinityOS → “TrinityOS”  
• TrinitySearch → “TrinitySearch”  
• TrinityAngleSolver → “TrinityAngleSolver”  
• TrinityPolynomial → “TrinityPolynomial”  
• TrinityPolynomial2 → “TrinityPolynomial2”  
• TrinityOne → “TrinityOne”  
• TrinityTwo → “TrinityTwo”  
• TrinityThree → “TrinityThree”  
• TrinityFour → “TrinityFour”

Make sure every script matches its housing or nothing will work correctly.

----------------------------------------

[h2]Step 5 – Power the System On (Except TrinityOS)[/h2]

1. Turn ON all chips EXCEPT TrinityOS.  
2. Apply power to the heavy cable network.

System may draw up to ~50,000w.

----------------------------------------

[h2]Step 6 – Initialize TrinityOS[/h2]

1. Double-check all naming: Housings + Buttons.  
2. Turn on the Large LED Display.  
3. Insert the Comms Motherboard into the computer.  
4. Turn ON the TrinityOS chip.

If everything is correct you will see:
OS Nominal

If you get:
Error Check Housings

…then one or more housings are misnamed or scripts are missing. Fix and try again.

----------------------------------------

[h2]Step 7 – Operating TrinityOS[/h2]

After boot:

• Select Trader Tier (0 = close, 1 = mid, 2 = far) using the Dial  
• Press Enter to confirm  
• Follow LED instructions

When prompted:
Interrogate?
Call Down?

Press:
• Yes = proceed  
• No = reset (do NOT press Enter for these)

After calling down a trader, the OS will automatically reset and you can start again.

----------------------------------------

[h1]Happy Hunting![/h1]
