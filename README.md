# Dofbot
- **main**:answer from git tree

##### **target** 
###### ** target_pos**:
 - 使用deepcopy()函数对block_pos进行深拷贝，确保数据的独立性。

 ##### **eular_pos**:
 - 通过调用 R.from_quat(target_orn)，我们可以将四元数 target_orn 转换为 Rotation 对象，然后调用 as_euler("xyz", degrees=False) 将其转换为欧拉角表示，并将结果存储在 eular_pos。
 