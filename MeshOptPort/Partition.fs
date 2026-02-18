// This file is part of MeshOptPort; see meshoptimizer.h for version/license details
module MeshOptPort.Partition

// This work is based on:
// Takio Kurita. An efficient agglomerative clustering algorithm using a heap. 1991

#nowarn "9"

open System
open FSharp.NativeInterop
open MeshOptPort.Allocator

// To avoid excessive recursion for malformed inputs, we switch to bisection after some depth
[<Literal>]
let private kMergeDepthCutoff = 40

type private ClusterAdjacency = {
    mutable offsets: nativeptr<uint32>
    mutable clusters: nativeptr<uint32>
    mutable shared: nativeptr<uint32>
}

[<Struct>]
type private ClusterGroup =
    val mutable group: int
    val mutable next: int
    val mutable size: uint32
    val mutable vertices: uint32
    val mutable center_0: float32
    val mutable center_1: float32
    val mutable center_2: float32
    val mutable radius: float32

[<Struct>]
type private GroupOrder =
    val mutable id: uint32
    val mutable order: int

let private filterClusterIndices (data: nativeptr<uint32>) (offsets: nativeptr<uint32>) (cluster_indices: nativeptr<uint32>) (cluster_index_counts: nativeptr<uint32>) (cluster_count: int) (used: nativeptr<byte>) (vertex_count: int) (total_index_count: int) =
    let mutable cluster_start = 0
    let mutable cluster_write = 0

    for i = 0 to cluster_count - 1 do
        NPtr.set offsets i (uint32 cluster_write)

        // copy cluster indices, skipping duplicates
        for j = 0 to int (NPtr.get cluster_index_counts i) - 1 do
            let v = NPtr.get cluster_indices (cluster_start + j)
            assert (int v < vertex_count)

            NPtr.set data cluster_write v
            cluster_write <- cluster_write + 1 - int (NPtr.get used (int v))
            NPtr.set used (int v) 1uy

        // reset used flags for the next cluster
        let mutable j = int (NPtr.get offsets i)
        while j < cluster_write do
            NPtr.set used (int (NPtr.get data j)) 0uy
            j <- j + 1

        cluster_start <- cluster_start + int (NPtr.get cluster_index_counts i)

    assert (cluster_start = total_index_count)
    assert (cluster_write <= total_index_count)
    NPtr.set offsets cluster_count (uint32 cluster_write)

let private computeClusterBounds (indices: nativeptr<uint32>) (index_count: int) (vertex_positions: nativeptr<float32>) (vertex_positions_stride: int) (out_center: nativeptr<float32>) : float32 =
    let vertex_stride_float = vertex_positions_stride / sizeof<float32>

    let mutable center_0 = 0.0f
    let mutable center_1 = 0.0f
    let mutable center_2 = 0.0f

    // approximate center of the cluster by averaging all vertex positions
    for j = 0 to index_count - 1 do
        let pi = int (NPtr.get indices j) * vertex_stride_float
        center_0 <- center_0 + NPtr.get vertex_positions (pi + 0)
        center_1 <- center_1 + NPtr.get vertex_positions (pi + 1)
        center_2 <- center_2 + NPtr.get vertex_positions (pi + 2)

    // note: technically clusters can't be empty per meshopt_partitionCluster but we check for a division by zero in case that changes
    if index_count > 0 then
        let fic = float32 index_count
        center_0 <- center_0 / fic
        center_1 <- center_1 / fic
        center_2 <- center_2 / fic

    // compute radius of the bounding sphere for each cluster
    let mutable radiussq = 0.0f

    for j = 0 to index_count - 1 do
        let pi = int (NPtr.get indices j) * vertex_stride_float
        let p0 = NPtr.get vertex_positions (pi + 0)
        let p1 = NPtr.get vertex_positions (pi + 1)
        let p2 = NPtr.get vertex_positions (pi + 2)

        let d2 = (p0 - center_0) * (p0 - center_0) + (p1 - center_1) * (p1 - center_1) + (p2 - center_2) * (p2 - center_2)

        if d2 > radiussq then radiussq <- d2

    NPtr.set out_center 0 center_0
    NPtr.set out_center 1 center_1
    NPtr.set out_center 2 center_2
    MathF.Sqrt(radiussq)

let private buildClusterAdjacency (adjacency: ClusterAdjacency) (cluster_indices: nativeptr<uint32>) (cluster_offsets: nativeptr<uint32>) (cluster_count: int) (vertex_count: int) (allocator: meshopt_Allocator) =
    let ref_offsets = allocator.allocate<uint32>(vertex_count + 1)

    // compute number of clusters referenced by each vertex
    NPtr.memset ref_offsets 0uy (vertex_count * sizeof<uint32>)

    for i = 0 to cluster_count - 1 do
        let mutable j = int (NPtr.get cluster_offsets i)
        while j < int (NPtr.get cluster_offsets (i + 1)) do
            let v = int (NPtr.get cluster_indices j)
            NPtr.set ref_offsets v (NPtr.get ref_offsets v + 1u)
            j <- j + 1

    // compute (worst-case) number of adjacent clusters for each cluster
    let mutable total_adjacency = 0

    for i = 0 to cluster_count - 1 do
        let mutable count = 0

        // worst case is every vertex has a disjoint cluster list
        let mutable j = int (NPtr.get cluster_offsets i)
        while j < int (NPtr.get cluster_offsets (i + 1)) do
            count <- count + int (NPtr.get ref_offsets (int (NPtr.get cluster_indices j))) - 1
            j <- j + 1

        // ... but only every other cluster can be adjacent in the end
        total_adjacency <- total_adjacency + (if count < cluster_count - 1 then count else cluster_count - 1)

    // we can now allocate adjacency buffers
    adjacency.offsets <- allocator.allocate<uint32>(cluster_count + 1)
    adjacency.clusters <- allocator.allocate<uint32>(total_adjacency)
    adjacency.shared <- allocator.allocate<uint32>(total_adjacency)

    // convert ref counts to offsets
    let mutable total_refs = 0

    for i = 0 to vertex_count - 1 do
        let count = int (NPtr.get ref_offsets i)
        NPtr.set ref_offsets i (uint32 total_refs)
        total_refs <- total_refs + count

    let ref_data = allocator.allocate<uint32>(total_refs)

    // fill cluster refs for each vertex
    for i = 0 to cluster_count - 1 do
        let mutable j = int (NPtr.get cluster_offsets i)
        while j < int (NPtr.get cluster_offsets (i + 1)) do
            let v = int (NPtr.get cluster_indices j)
            let off = int (NPtr.get ref_offsets v)
            NPtr.set ref_data off (uint32 i)
            NPtr.set ref_offsets v (NPtr.get ref_offsets v + 1u)
            j <- j + 1

    // after the previous pass, ref_offsets contain the end of the data for each vertex; shift it forward to get the start
    // memmove(ref_offsets + 1, ref_offsets, vertex_count * sizeof(unsigned int)) — overlapping backward copy
    let byteCount = int64 (vertex_count * sizeof<uint32>)
    System.Buffer.MemoryCopy(NPtr.toVoid ref_offsets, NPtr.toVoid (NPtr.add ref_offsets 1), byteCount, byteCount)
    NPtr.set ref_offsets 0 0u

    // fill cluster adjacency for each cluster...
    NPtr.set adjacency.offsets 0 0u

    for i = 0 to cluster_count - 1 do
        let adj = NPtr.add adjacency.clusters (int (NPtr.get adjacency.offsets i))
        let shd = NPtr.add adjacency.shared (int (NPtr.get adjacency.offsets i))
        let mutable count = 0

        let mutable j = int (NPtr.get cluster_offsets i)
        while j < int (NPtr.get cluster_offsets (i + 1)) do
            let v = int (NPtr.get cluster_indices j)

            // merge the entire cluster list of each vertex into current list
            let mutable k = int (NPtr.get ref_offsets v)
            while k < int (NPtr.get ref_offsets (v + 1)) do
                let c = NPtr.get ref_data k
                assert (int c < cluster_count)

                if c <> uint32 i then
                    // if the cluster is already in the list, increment the shared count
                    let mutable found = false
                    let mutable l = 0
                    while l < count && not found do
                        if NPtr.get adj l = c then
                            found <- true
                            NPtr.set shd l (NPtr.get shd l + 1u)
                        l <- l + 1

                    // .. or append a new cluster
                    if not found then
                        NPtr.set adj count c
                        NPtr.set shd count 1u
                        count <- count + 1

                k <- k + 1
            j <- j + 1

        // mark the end of the adjacency list; the next cluster will start there as well
        NPtr.set adjacency.offsets (i + 1) (NPtr.get adjacency.offsets i + uint32 count)

    assert (int (NPtr.get adjacency.offsets cluster_count) <= total_adjacency)

    // ref_offsets can't be deallocated as it was allocated before adjacency
    allocator.deallocate(NPtr.toNI ref_data)

let private heapPush (heap: nativeptr<GroupOrder>) (size: int) (item: GroupOrder) =
    let mutable sz = size
    NPtr.set heap sz item
    sz <- sz + 1

    // bubble up the new element to its correct position
    let mutable i = sz - 1
    let mutable cont = true
    while cont && i > 0 do
        let p = (i - 1) / 2
        if (NPtr.get heap i).order < (NPtr.get heap p).order then
            let temp = NPtr.get heap i
            NPtr.set heap i (NPtr.get heap p)
            NPtr.set heap p temp
            i <- p
        else
            cont <- false

let private heapPop (heap: nativeptr<GroupOrder>) (size: int) : GroupOrder =
    assert (size > 0)
    let top = NPtr.get heap 0
    let mutable sz = size - 1

    // move the last element to the top (breaks heap invariant)
    NPtr.set heap 0 (NPtr.get heap sz)

    // bubble down the new top element to its correct position
    let mutable i = 0
    let mutable cont = true
    while cont && i * 2 + 1 < sz do
        // find the smallest child
        let mutable j = i * 2 + 1
        if j + 1 < sz && (NPtr.get heap (j + 1)).order < (NPtr.get heap j).order then
            j <- j + 1

        // if the parent is already smaller than both children, we're done
        if (NPtr.get heap j).order >= (NPtr.get heap i).order then
            cont <- false
        else
            // otherwise, swap the parent and child and continue
            let temp = NPtr.get heap i
            NPtr.set heap i (NPtr.get heap j)
            NPtr.set heap j temp
            i <- j

    top

let private countShared (groups: nativeptr<ClusterGroup>) (group1: int) (group2: int) (adjacency: ClusterAdjacency) : uint32 =
    let mutable total = 0u

    let mutable i1 = group1
    while i1 >= 0 do
        let mutable i2 = group2
        while i2 >= 0 do
            let mutable adj = int (NPtr.get adjacency.offsets i1)
            while adj < int (NPtr.get adjacency.offsets (i1 + 1)) do
                if NPtr.get adjacency.clusters adj = uint32 i2 then
                    total <- total + NPtr.get adjacency.shared adj
                    adj <- int (NPtr.get adjacency.offsets (i1 + 1)) // break
                else
                    adj <- adj + 1
            i2 <- (NPtr.get groups i2).next
        i1 <- (NPtr.get groups i1).next

    total

let private mergeBounds (target: nativeptr<ClusterGroup>) (targetIdx: int) (source: nativeptr<ClusterGroup>) (sourceIdx: int) =
    let t = NPtr.get target targetIdx
    let s = NPtr.get source sourceIdx
    let r1 = t.radius
    let r2 = s.radius
    let dx = s.center_0 - t.center_0
    let dy = s.center_1 - t.center_1
    let dz = s.center_2 - t.center_2
    let d = MathF.Sqrt(dx * dx + dy * dy + dz * dz)

    if d + r1 < r2 then
        let mutable tg = NPtr.get target targetIdx
        tg.center_0 <- s.center_0
        tg.center_1 <- s.center_1
        tg.center_2 <- s.center_2
        tg.radius <- s.radius
        NPtr.set target targetIdx tg
    elif d + r2 > r1 then
        let k = if d > 0.0f then (d + r2 - r1) / (2.0f * d) else 0.0f
        let mutable tg = NPtr.get target targetIdx
        tg.center_0 <- tg.center_0 + dx * k
        tg.center_1 <- tg.center_1 + dy * k
        tg.center_2 <- tg.center_2 + dz * k
        tg.radius <- (d + r2 + r1) / 2.0f
        NPtr.set target targetIdx tg

let private boundsScore (target: nativeptr<ClusterGroup>) (targetIdx: int) (source: nativeptr<ClusterGroup>) (sourceIdx: int) : float32 =
    let t = NPtr.get target targetIdx
    let s = NPtr.get source sourceIdx
    let r1 = t.radius
    let r2 = s.radius
    let dx = s.center_0 - t.center_0
    let dy = s.center_1 - t.center_1
    let dz = s.center_2 - t.center_2
    let d = MathF.Sqrt(dx * dx + dy * dy + dz * dz)

    let mr =
        if d + r1 < r2 then r2
        elif d + r2 < r1 then r1
        else (d + r2 + r1) / 2.0f

    if mr > 0.0f then r1 / mr else 0.0f

let private pickGroupToMerge (groups: nativeptr<ClusterGroup>) (id: int) (adjacency: ClusterAdjacency) (max_partition_size: int) (use_bounds: bool) : int =
    assert ((NPtr.get groups id).size > 0u)

    let group_rsqrt = 1.0f / MathF.Sqrt(float32 (int (NPtr.get groups id).vertices))

    let mutable best_group = -1
    let mutable best_score = 0.0f

    let mutable ci = id
    while ci >= 0 do
        let mutable adj = int (NPtr.get adjacency.offsets ci)
        while adj < int (NPtr.get adjacency.offsets (ci + 1)) do
            let other = (NPtr.get groups (int (NPtr.get adjacency.clusters adj))).group
            if other >= 0 then
                assert ((NPtr.get groups other).size > 0u)
                if int (NPtr.get groups id).size + int (NPtr.get groups other).size <= max_partition_size then
                    let shared = countShared groups id other adjacency
                    let other_rsqrt = 1.0f / MathF.Sqrt(float32 (int (NPtr.get groups other).vertices))

                    // normalize shared count by the expected boundary of each group (+ keeps scoring symmetric)
                    let mutable score = float32 (int shared) * (group_rsqrt + other_rsqrt)

                    // incorporate spatial score to favor merging nearby groups
                    if use_bounds then
                        score <- score * (1.0f + 0.4f * boundsScore groups id groups other)

                    if score > best_score then
                        best_group <- other
                        best_score <- score
            adj <- adj + 1
        ci <- (NPtr.get groups ci).next

    best_group

let private mergeLeaf (groups: nativeptr<ClusterGroup>) (order: nativeptr<uint32>) (count: int) (target_partition_size: int) (max_partition_size: int) =
    for i = 0 to count - 1 do
        let id = int (NPtr.get order i)
        if (NPtr.get groups id).size <> 0u && (NPtr.get groups id).size < uint32 target_partition_size then
            let mutable best_score = -1.0f
            let mutable best_group = -1

            for j = 0 to count - 1 do
                let other = int (NPtr.get order j)
                if id <> other && (NPtr.get groups other).size <> 0u then
                    if int (NPtr.get groups id).size + int (NPtr.get groups other).size <= max_partition_size then
                        // favor merging nearby groups
                        let score = boundsScore groups id groups other

                        if score > best_score then
                            best_score <- score
                            best_group <- other

            // merge id *into* best_group; that way, we may merge more groups into the same best_group, maximizing the chance of reaching target
            if best_group <> -1 then
                // combine groups by linking them together
                let mutable tail = best_group
                while (NPtr.get groups tail).next >= 0 do
                    tail <- (NPtr.get groups tail).next

                let mutable g = NPtr.get groups tail
                g.next <- id
                NPtr.set groups tail g

                // update group sizes; note, we omit vertices update for simplicity as it's not used for spatial merge
                let mutable bg = NPtr.get groups best_group
                bg.size <- bg.size + (NPtr.get groups id).size
                NPtr.set groups best_group bg

                let mutable ig = NPtr.get groups id
                ig.size <- 0u
                NPtr.set groups id ig

                // merge bounding spheres
                mergeBounds groups best_group groups id

                let mutable ig2 = NPtr.get groups id
                ig2.radius <- 0.0f
                NPtr.set groups id ig2

let private mergePartition (order: nativeptr<uint32>) (count: int) (groups: nativeptr<ClusterGroup>) (axis: int) (pivot: float32) : int =
    let mutable m = 0

    // invariant: elements in range [0, m) are < pivot, elements in range [m, i) are >= pivot
    for i = 0 to count - 1 do
        let g = NPtr.get groups (int (NPtr.get order i))
        let v =
            match axis with
            | 0 -> g.center_0
            | 1 -> g.center_1
            | _ -> g.center_2

        // swap(m, i) unconditionally
        let t = NPtr.get order m
        NPtr.set order m (NPtr.get order i)
        NPtr.set order i t

        // when v >= pivot, we swap i with m without advancing it, preserving invariants
        if v < pivot then m <- m + 1

    m

let rec private mergeSpatial (groups: nativeptr<ClusterGroup>) (order: nativeptr<uint32>) (count: int) (target_partition_size: int) (max_partition_size: int) (leaf_size: int) (depth: int) =
    let mutable total = 0
    for i = 0 to count - 1 do
        total <- total + int (NPtr.get groups (int (NPtr.get order i))).size

    if total <= max_partition_size || count <= leaf_size then
        mergeLeaf groups order count target_partition_size max_partition_size
    else
        let mutable mean_0 = 0.0f
        let mutable mean_1 = 0.0f
        let mutable mean_2 = 0.0f
        let mutable vars_0 = 0.0f
        let mutable vars_1 = 0.0f
        let mutable vars_2 = 0.0f
        let mutable runc = 1.0f

        // gather statistics on the points in the subtree using Welford's algorithm
        for i = 0 to count - 1 do
            let runs = 1.0f / runc
            let g = NPtr.get groups (int (NPtr.get order i))

            let delta0 = g.center_0 - mean_0
            mean_0 <- mean_0 + delta0 * runs
            vars_0 <- vars_0 + delta0 * (g.center_0 - mean_0)

            let delta1 = g.center_1 - mean_1
            mean_1 <- mean_1 + delta1 * runs
            vars_1 <- vars_1 + delta1 * (g.center_1 - mean_1)

            let delta2 = g.center_2 - mean_2
            mean_2 <- mean_2 + delta2 * runs
            vars_2 <- vars_2 + delta2 * (g.center_2 - mean_2)

            runc <- runc + 1.0f

        // split axis is one where the variance is largest
        let axis =
            if vars_0 >= vars_1 && vars_0 >= vars_2 then 0
            elif vars_1 >= vars_2 then 1
            else 2

        let split =
            match axis with
            | 0 -> mean_0
            | 1 -> mean_1
            | _ -> mean_2

        let mutable middle = mergePartition order count groups axis split

        // enforce balance for degenerate partitions
        // this also ensures recursion depth is bounded on pathological inputs
        if middle <= leaf_size / 2 || count - middle <= leaf_size / 2 || depth >= kMergeDepthCutoff then
            middle <- count / 2

        // recursion depth is logarithmic and bounded due to max depth check above
        mergeSpatial groups order middle target_partition_size max_partition_size leaf_size (depth + 1)
        mergeSpatial groups (NPtr.add order middle) (count - middle) target_partition_size max_partition_size leaf_size (depth + 1)

let meshopt_partitionClusters (destination: nativeptr<uint32>) (cluster_indices: nativeptr<uint32>) (total_index_count: int) (cluster_index_counts: nativeptr<uint32>) (cluster_count: int) (vertex_positions: nativeptr<float32>) (vertex_count: int) (vertex_positions_stride: int) (target_partition_size: int) : int =
    assert ((NPtr.toNI vertex_positions = 0n || vertex_positions_stride >= 12) && vertex_positions_stride <= 256)
    assert (vertex_positions_stride % sizeof<float32> = 0)
    assert (target_partition_size > 0)

    let max_partition_size = target_partition_size + target_partition_size / 3

    use allocator = new meshopt_Allocator()

    let used = allocator.allocate<byte>(vertex_count)
    NPtr.memset used 0uy vertex_count

    let cluster_newindices = allocator.allocate<uint32>(total_index_count)
    let cluster_offsets = allocator.allocate<uint32>(cluster_count + 1)

    // make new cluster index list that filters out duplicate indices
    filterClusterIndices cluster_newindices cluster_offsets cluster_indices cluster_index_counts cluster_count used vertex_count total_index_count
    let cluster_indices = cluster_newindices

    // build cluster adjacency along with edge weights (shared vertex count)
    let adjacency = { offsets = NPtr.ofNI 0n; clusters = NPtr.ofNI 0n; shared = NPtr.ofNI 0n }
    buildClusterAdjacency adjacency cluster_indices cluster_offsets cluster_count vertex_count allocator

    let groups = allocator.allocate<ClusterGroup>(cluster_count)
    NPtr.memset groups 0uy (sizeof<ClusterGroup> * cluster_count)

    let order = allocator.allocate<GroupOrder>(cluster_count)
    let mutable pending = 0

    let has_positions = NPtr.toNI vertex_positions <> 0n

    // temporary buffer for center computation
    let center_buf = NativePtr.stackalloc<float32> 3

    // create a singleton group for each cluster and order them by priority
    for i = 0 to cluster_count - 1 do
        let mutable g = ClusterGroup()
        g.group <- i
        g.next <- -1
        g.size <- 1u
        g.vertices <- NPtr.get cluster_offsets (i + 1) - NPtr.get cluster_offsets i
        assert (g.vertices > 0u)

        // compute bounding sphere for each cluster if positions are provided
        if has_positions then
            g.radius <- computeClusterBounds (NPtr.add cluster_indices (int (NPtr.get cluster_offsets i))) (int (NPtr.get cluster_offsets (i + 1) - NPtr.get cluster_offsets i)) vertex_positions vertex_positions_stride center_buf
            g.center_0 <- NPtr.get center_buf 0
            g.center_1 <- NPtr.get center_buf 1
            g.center_2 <- NPtr.get center_buf 2

        NPtr.set groups i g

        let mutable item = GroupOrder()
        item.id <- uint32 i
        item.order <- int g.vertices

        heapPush order pending item
        pending <- pending + 1

    // iteratively merge the smallest group with the best group
    while pending > 0 do
        let top = heapPop order pending
        pending <- pending - 1

        // this group was merged into another group earlier
        if (NPtr.get groups (int top.id)).size <> 0u then
            // disassociate clusters from the group to prevent them from being merged again; we will re-associate them if the group is reinserted
            let mutable ci = int top.id
            while ci >= 0 do
                assert ((NPtr.get groups ci).group = int top.id)
                let mutable g = NPtr.get groups ci
                g.group <- -1
                NPtr.set groups ci g
                ci <- (NPtr.get groups ci).next

            // the group is large enough, emit as is
            if (NPtr.get groups (int top.id)).size < uint32 target_partition_size then
                let best_group = pickGroupToMerge groups (int top.id) adjacency max_partition_size has_positions

                // we can't grow the group any more, emit as is
                if best_group <> -1 then
                    // compute shared vertices to adjust the total vertices estimate after merging
                    let shared = countShared groups (int top.id) best_group adjacency

                    // combine groups by linking them together
                    let mutable tail = int top.id
                    while (NPtr.get groups tail).next >= 0 do
                        tail <- (NPtr.get groups tail).next

                    let mutable tg = NPtr.get groups tail
                    tg.next <- best_group
                    NPtr.set groups tail tg

                    // update group sizes; note, the vertex update is a O(1) approximation which avoids recomputing the true size
                    let mutable topg = NPtr.get groups (int top.id)
                    topg.size <- topg.size + (NPtr.get groups best_group).size
                    topg.vertices <- topg.vertices + (NPtr.get groups best_group).vertices
                    topg.vertices <- if topg.vertices > shared then topg.vertices - shared else 1u
                    NPtr.set groups (int top.id) topg

                    let mutable bg = NPtr.get groups best_group
                    bg.size <- 0u
                    bg.vertices <- 0u
                    NPtr.set groups best_group bg

                    // merge bounding spheres if bounds are available
                    if has_positions then
                        mergeBounds groups (int top.id) groups best_group
                        let mutable bg2 = NPtr.get groups best_group
                        bg2.radius <- 0.0f
                        NPtr.set groups best_group bg2

                    // re-associate all clusters back to the merged group
                    let mutable ri = int top.id
                    while ri >= 0 do
                        let mutable rg = NPtr.get groups ri
                        rg.group <- int top.id
                        NPtr.set groups ri rg
                        ri <- (NPtr.get groups ri).next

                    let mutable newTop = top
                    newTop.order <- int (NPtr.get groups (int top.id)).vertices
                    heapPush order pending newTop
                    pending <- pending + 1

    // if vertex positions are provided, we do a final pass to see if we can merge small groups based on spatial locality alone
    if has_positions then
        let merge_order : nativeptr<uint32> = NPtr.cast<GroupOrder, uint32> order
        let mutable merge_offset = 0

        for i = 0 to cluster_count - 1 do
            if (NPtr.get groups i).size > 0u then
                NPtr.set merge_order merge_offset (uint32 i)
                merge_offset <- merge_offset + 1

        mergeSpatial groups merge_order merge_offset target_partition_size max_partition_size 8 0

    // output each remaining group
    let mutable next_group = 0

    for i = 0 to cluster_count - 1 do
        if (NPtr.get groups i).size <> 0u then
            let mutable j = i
            while j >= 0 do
                NPtr.set destination j (uint32 next_group)
                j <- (NPtr.get groups j).next

            next_group <- next_group + 1

    assert (next_group <= cluster_count)
    next_group
