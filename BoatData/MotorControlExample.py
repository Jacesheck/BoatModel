import matplotlib.pyplot as plt
import numpy as np

def normalize(input1, input2, maxi):
    total = abs(input1) + abs(input2)
    if total > maxi:
        input_max = max(abs(input1), abs(input2))
        input1 *= maxi / input_max
        input2 *= maxi / input_max
    return input1, input2

originalNormal = np.array([100, 100]) 
originalTurn   = np.array([30, -30])

barWidth = 0.25

br_normal = np.arange(len(originalNormal)) + 0.75 + 0.25/2 
br_turn   = br_normal + barWidth
print(br_normal)
print(br_turn)

plt.subplot(1, 3, 1)
plt.title('Individual control inputs')
plt.xlabel('Motor')
plt.ylabel('Power')
plt.plot([0.75, 2.25], [100, 100], 'k-.', label='Min/Max')
plt.plot([0.75, 2.25], [0, 0], 'b-')
plt.plot([0.75, 2.25], [-100, -100], 'k-.')
plt.bar(br_normal, originalNormal, width=barWidth, label='Normal power')
plt.bar(br_turn, originalTurn, width=barWidth, label='Turn power')
plt.ylim((-150, 150))
plt.legend()

added_left = originalNormal[0] + originalTurn[0]
added_right = originalNormal[1] + originalTurn[1]
plt.subplot(1, 3, 2)
plt.title('Added power')
plt.xlabel('Motor')
plt.plot([0.75, 2.25], [100, 100], 'k-.', label='Min/Max')
plt.plot([0.75, 2.25], [0, 0], 'b-')
plt.plot([0.75, 2.25], [-100, -100], 'k-.')
plt.bar([1, 2], [added_left, added_right], width=barWidth, label='Power')
plt.ylim((-150, 150))
plt.legend()

left, right = normalize(added_left, added_right, 100)
plt.subplot(1, 3, 3)
plt.title('Normalised power')
plt.xlabel('Motor')
plt.plot([0.75, 2.25], [100, 100], 'k-.', label='Min/Max')
plt.plot([0.75, 2.25], [0, 0], 'b-')
plt.plot([0.75, 2.25], [-100, -100], 'k-.')
plt.bar([1, 2], [left, right], width=barWidth, label='Power')
plt.ylim((-150, 150))
plt.legend()
plt.show()
