#ifndef __DFS_H
#define __DFS_H

#define NODE_COUNT 14

// 全局边矩阵，由调用者初始化，edge[from][to] 的值表示方向编码（1,2,3,4）
extern int edge[NODE_COUNT][NODE_COUNT];

/*
 * 函数: dfs
 * 说明: 在全局 edge 图中查找从 start 到 goal 的路径，
 *       并将路径以字符序列存储到 result 数组中（调用者须保证 result 的空间足够）。
 *       方向映射：1->'U'（上），2->'L'（左），3->'D'（下），4->'R'（右）。
 *       若无路径，则 result[0] 置为 '\0'。
 */
void dfs(int start, int goal, char *result);

#endif
