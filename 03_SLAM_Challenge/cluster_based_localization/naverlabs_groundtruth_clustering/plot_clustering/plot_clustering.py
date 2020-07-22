from sklearn.cluster import AgglomerativeClustering

from matplotlib import pyplot as plt
from matplotlib import patches as mpatches

import numpy as np

class cluster_plotter():

    @staticmethod
    def draw(X, labels, title=None):

        x_min, x_max = np.min(X, axis=0), np.max(X, axis=0)
        X = (X - x_min) / (x_max - x_min)

        plt.figure(figsize=(6,4))

        color_map = []
        legend_mat = []
        for i in range(int(max(labels))+1):
            color_map.append((np.random.rand(1)[0], np.random.rand(1)[0], np.random.rand(1)[0]))

            legend_mat.append(mpatches.Patch(color=color_map[i], label="Cluster" + str(i)))

        for i in range(X.shape[0]):

            plt.text(X[i,0], X[i,1], str(labels[i]),
                    color=color_map[labels[i]],
                    fontdict={'weight' : 'bold', 'size' : 9}
            )

        plt.xticks([])
        plt.yticks([])

        plt.legend(
            bbox_to_anchor=(0, -0.05),
            handles=legend_mat,
            loc='upper left',
            ncol=10,
            fancybox=True
        )

        if title is not None:
            plt.title(title, size=17)

        plt.axis('off')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        plt.show()