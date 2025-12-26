# ANALYSIS: Polynomial Correction Impact on Spiral Coverage
# 
# Problem: Your spiral scans in COMMANDED angles, but polynomial correction
# means ACTUAL pointing angles are very different, creating coverage gaps.
#
# Commanded V=48° → Actual V=39.25° (8.75° error!)
# Trader at Actual V=48° requires Commanded V≈57°
#
# Your 0.9° H / 0.06875° V pattern in COMMANDED space becomes:
# - Variable actual vertical increment (0.025° to 0.085° depending on V)
# - This creates irregular spiral spacing in actual pointing space
# - Gaps appear where vertical increment is compressed
#
# SOLUTION OPTIONS:
#
# Option 1: ADAPTIVE COMMANDED INCREMENTS
# Calculate commanded increments that produce constant actual increments:
#   - At V=0-30° (low offset): Use larger commanded V increment
#   - At V=30-60° (high offset): Use smaller commanded V increment  
#   - At V=60-90° (decreasing offset): Use larger commanded V increment
#
# Option 2: INVERTED LOOKUP
# Work in actual angle space:
#   1. Plan spiral in actual V (0° to 89.99° actual)
#   2. For each actual V, look up required commanded V
#   3. Send commanded angles to dish
#
# Option 3: SIMPLER FIX
# Just reduce H increment to 0.25° regardless of polynomial
# This gives 3.6x more passes, compensating for vertical irregularity
#
# RECOMMENDED: Option 3 (simplest, robust)
# Even with polynomial warping, 0.25° H gives enough coverage density
# to catch traders in the gaps created by vertical compression.

# INVERTED LOOKUP TABLE (for Option 2 if needed)
# Maps: Actual V → Commanded V
# Generated from your polynomial data

# Actual V : Commanded V
#   0.01   :   0
#   5.00   :  10
#  10.00   :  17
#  15.00   :  23
#  20.00   :  28
#  25.00   :  32
#  30.00   :  36
#  35.00   :  39
#  40.00   :  43
#  45.00   :  46
#  48.00   :  48  (Your missed trader needs V_cmd ≈ 57!)
#  50.00   :  50
#  55.00   :  53
#  60.00   :  57
#  65.00   :  62
#  70.00   :  68
#  75.00   :  74
#  80.00   :  81
#  85.00   :  88
#  89.99   :  90

# DIAGNOSIS OF YOUR MISSED TRADER:
# Trader actual position: H=123°, V=48° (actual)
# To point at actual V=48°, you need commanded V≈57°
# 
# Your spiral in commanded space:
#   Reaches commanded V=48° at step 698 (48÷0.06875)
#   At this point, H = 698×0.9 = 628.2° = 268.2° (wrapped)
#   Actual V at commanded V=48° = 39.25° (from table)
#   
# So when your spiral was at H=268°, V=39° (actual), the trader at
# H=123°, V=48° (actual) was 9° above you vertically.
#
# When your spiral reached actual V=48° (at commanded V≈57°):
#   Step number: 830 (57÷0.06875)
#   H = 830×0.9 = 747° = 27° (wrapped)
#   The trader at H=123° was 96° away horizontally - MISSED!

# VISUALIZATION DATA FOR YOUR SPECIFIC MISS:
# The trader appears to be "between spiral loops" because:
# 1. The polynomial compression reduces your vertical coverage rate
# 2. By the time you reach the trader's actual V, you've wrapped around H
# 3. The 0.9° H increment means 13° gaps between passes
# 4. Trader fell into this gap

print("See analysis above")
print("Recommendation: Change H increment from 0.9° to 0.25° in Trinity Search code")
