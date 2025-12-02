/**
 * @file matrix_math.c
 * @brief Implementation of general-purpose matrix mathematics library
 */

#include "matrix_math.h"
#include <math.h>
#include <string.h>

/*******************************************************************************
 * Matrix Creation and Initialization
 ******************************************************************************/

int8_t matrixInit(Matrix_t *mat, uint8_t rows, uint8_t cols)
{
    if (mat == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    if (rows == 0 || cols == 0 || rows > MATRIX_MAX_SIZE || cols > MATRIX_MAX_SIZE) {
        return MATRIX_ERR_INVALID_SIZE;
    }
    
    mat->rows = rows;
    mat->cols = cols;
    
    /* Zero initialize all elements */
    uint16_t total_elements = (uint16_t)rows * cols;
    for (uint16_t i = 0; i < total_elements; i++) {
        mat->data[i] = 0.0f;
    }
    
    return MATRIX_OK;
}

int8_t matrixEye(Matrix_t *mat, uint8_t size)
{
    if (mat == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    if (size == 0 || size > MATRIX_MAX_SIZE) {
        return MATRIX_ERR_INVALID_SIZE;
    }
    
    /* Initialize to zeros first */
    int8_t ret = matrixInit(mat, size, size);
    if (ret != MATRIX_OK) {
        return ret;
    }
    
    /* Set diagonal elements to 1 */
    for (uint8_t i = 0; i < size; i++) {
        mat->data[i * size + i] = 1.0f;
    }
    
    return MATRIX_OK;
}

int8_t matrixFill(Matrix_t *mat, float value)
{
    if (mat == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    uint16_t total_elements = (uint16_t)mat->rows * mat->cols;
    for (uint16_t i = 0; i < total_elements; i++) {
        mat->data[i] = value;
    }
    
    return MATRIX_OK;
}

int8_t matrixSetData(Matrix_t *mat, const float *data, uint8_t rows, uint8_t cols)
{
    if (mat == NULL || data == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    if (rows == 0 || cols == 0 || rows > MATRIX_MAX_SIZE || cols > MATRIX_MAX_SIZE) {
        return MATRIX_ERR_INVALID_SIZE;
    }
    
    mat->rows = rows;
    mat->cols = cols;
    
    uint16_t total_elements = (uint16_t)rows * cols;
    memcpy(mat->data, data, total_elements * sizeof(float));
    
    return MATRIX_OK;
}

int8_t matrixDiag(Matrix_t *mat, const float *diag, uint8_t size)
{
    if (mat == NULL || diag == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    /* Initialize to zeros first */
    int8_t ret = matrixInit(mat, size, size);
    if (ret != MATRIX_OK) {
        return ret;
    }
    
    /* Set diagonal elements */
    for (uint8_t i = 0; i < size; i++) {
        mat->data[i * size + i] = diag[i];
    }
    
    return MATRIX_OK;
}

/*******************************************************************************
 * Element Access
 ******************************************************************************/

int8_t matrixGet(const Matrix_t *mat, uint8_t row, uint8_t col, float *value)
{
    if (mat == NULL || value == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    if (row >= mat->rows || col >= mat->cols) {
        return MATRIX_ERR_OUT_OF_BOUNDS;
    }
    
    *value = mat->data[row * mat->cols + col];
    return MATRIX_OK;
}

int8_t matrixSet(Matrix_t *mat, uint8_t row, uint8_t col, float value)
{
    if (mat == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    if (row >= mat->rows || col >= mat->cols) {
        return MATRIX_ERR_OUT_OF_BOUNDS;
    }
    
    mat->data[row * mat->cols + col] = value;
    return MATRIX_OK;
}

/*******************************************************************************
 * Basic Matrix Operations
 ******************************************************************************/

int8_t matrixAdd(const Matrix_t *A, const Matrix_t *B, Matrix_t *result)
{
    if (A == NULL || B == NULL || result == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    if (A->rows != B->rows || A->cols != B->cols) {
        return MATRIX_ERR_DIM_MISMATCH;
    }
    
    result->rows = A->rows;
    result->cols = A->cols;
    
    uint16_t total_elements = (uint16_t)A->rows * A->cols;
    for (uint16_t i = 0; i < total_elements; i++) {
        result->data[i] = A->data[i] + B->data[i];
    }
    
    return MATRIX_OK;
}

int8_t matrixSub(const Matrix_t *A, const Matrix_t *B, Matrix_t *result)
{
    if (A == NULL || B == NULL || result == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    if (A->rows != B->rows || A->cols != B->cols) {
        return MATRIX_ERR_DIM_MISMATCH;
    }
    
    result->rows = A->rows;
    result->cols = A->cols;
    
    uint16_t total_elements = (uint16_t)A->rows * A->cols;
    for (uint16_t i = 0; i < total_elements; i++) {
        result->data[i] = A->data[i] - B->data[i];
    }
    
    return MATRIX_OK;
}

int8_t matrixMul(const Matrix_t *A, const Matrix_t *B, Matrix_t *result)
{
    if (A == NULL || B == NULL || result == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    /* Check dimension compatibility: A(m x n) * B(n x p) = result(m x p) */
    if (A->cols != B->rows) {
        return MATRIX_ERR_DIM_MISMATCH;
    }
    
    uint8_t m = A->rows;
    uint8_t n = A->cols;
    uint8_t p = B->cols;
    
    if (m > MATRIX_MAX_SIZE || p > MATRIX_MAX_SIZE) {
        return MATRIX_ERR_INVALID_SIZE;
    }
    
    result->rows = m;
    result->cols = p;
    
    /* Matrix multiplication: C[i][j] = sum(A[i][k] * B[k][j]) for k=0..n-1 */
    for (uint8_t i = 0; i < m; i++) {
        for (uint8_t j = 0; j < p; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < n; k++) {
                sum += A->data[i * n + k] * B->data[k * p + j];
            }
            result->data[i * p + j] = sum;
        }
    }
    
    return MATRIX_OK;
}

int8_t matrixScale(const Matrix_t *A, float scalar, Matrix_t *result)
{
    if (A == NULL || result == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    result->rows = A->rows;
    result->cols = A->cols;
    
    uint16_t total_elements = (uint16_t)A->rows * A->cols;
    for (uint16_t i = 0; i < total_elements; i++) {
        result->data[i] = A->data[i] * scalar;
    }
    
    return MATRIX_OK;
}

int8_t matrixTranspose(const Matrix_t *A, Matrix_t *result)
{
    if (A == NULL || result == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    uint8_t rows = A->rows;
    uint8_t cols = A->cols;
    
    result->rows = cols;
    result->cols = rows;
    
    /* Transpose: result[j][i] = A[i][j] */
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < cols; j++) {
            result->data[j * rows + i] = A->data[i * cols + j];
        }
    }
    
    return MATRIX_OK;
}

/*******************************************************************************
 * Advanced Matrix Operations
 ******************************************************************************/

int8_t matrixInverse(const Matrix_t *A, Matrix_t *result)
{
    if (A == NULL || result == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    /* Must be square matrix */
    if (A->rows != A->cols) {
        return MATRIX_ERR_DIM_MISMATCH;
    }
    
    uint8_t n = A->rows;
    
    /* Create augmented matrix [A | I] using temporary storage */
    float aug[MATRIX_MAX_SIZE][2 * MATRIX_MAX_SIZE];
    
    /* Initialize augmented matrix */
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            aug[i][j] = A->data[i * n + j];
            aug[i][j + n] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    /* Gauss-Jordan elimination */
    for (uint8_t col = 0; col < n; col++) {
        /* Find pivot (largest absolute value in column) */
        uint8_t pivot_row = col;
        float max_val = fabsf(aug[col][col]);
        
        for (uint8_t row = col + 1; row < n; row++) {
            float abs_val = fabsf(aug[row][col]);
            if (abs_val > max_val) {
                max_val = abs_val;
                pivot_row = row;
            }
        }
        
        /* Check for singularity */
        if (max_val < MATRIX_EPSILON) {
            return MATRIX_ERR_SINGULAR;
        }
        
        /* Swap rows if needed */
        if (pivot_row != col) {
            for (uint8_t j = 0; j < 2 * n; j++) {
                float temp = aug[col][j];
                aug[col][j] = aug[pivot_row][j];
                aug[pivot_row][j] = temp;
            }
        }
        
        /* Scale pivot row to make pivot = 1 */
        float pivot = aug[col][col];
        for (uint8_t j = 0; j < 2 * n; j++) {
            aug[col][j] /= pivot;
        }
        
        /* Eliminate column in all other rows */
        for (uint8_t row = 0; row < n; row++) {
            if (row != col) {
                float factor = aug[row][col];
                for (uint8_t j = 0; j < 2 * n; j++) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }
    }
    
    /* Extract inverse from right half of augmented matrix */
    result->rows = n;
    result->cols = n;
    
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            result->data[i * n + j] = aug[i][j + n];
        }
    }
    
    return MATRIX_OK;
}

int8_t matrixDeterminant(const Matrix_t *A, float *det)
{
    if (A == NULL || det == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    if (A->rows != A->cols) {
        return MATRIX_ERR_DIM_MISMATCH;
    }
    
    uint8_t n = A->rows;
    
    /* Special cases for small matrices */
    if (n == 1) {
        *det = A->data[0];
        return MATRIX_OK;
    }
    
    if (n == 2) {
        /* det = a*d - b*c for [a b; c d] */
        *det = A->data[0] * A->data[3] - A->data[1] * A->data[2];
        return MATRIX_OK;
    }
    
    if (n == 3) {
        /* Sarrus rule for 3x3 matrix */
        float a = A->data[0], b = A->data[1], c = A->data[2];
        float d = A->data[3], e = A->data[4], f = A->data[5];
        float g = A->data[6], h = A->data[7], i = A->data[8];
        
        *det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        return MATRIX_OK;
    }
    
    /* LU decomposition for larger matrices */
    float temp[MATRIX_MAX_SIZE][MATRIX_MAX_SIZE];
    float determinant = 1.0f;
    int8_t sign = 1;
    
    /* Copy matrix */
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            temp[i][j] = A->data[i * n + j];
        }
    }
    
    /* Gaussian elimination with partial pivoting */
    for (uint8_t col = 0; col < n; col++) {
        /* Find pivot */
        uint8_t pivot_row = col;
        float max_val = fabsf(temp[col][col]);
        
        for (uint8_t row = col + 1; row < n; row++) {
            if (fabsf(temp[row][col]) > max_val) {
                max_val = fabsf(temp[row][col]);
                pivot_row = row;
            }
        }
        
        if (max_val < MATRIX_EPSILON) {
            *det = 0.0f;
            return MATRIX_OK;
        }
        
        /* Swap rows if needed */
        if (pivot_row != col) {
            sign = -sign;
            for (uint8_t j = 0; j < n; j++) {
                float t = temp[col][j];
                temp[col][j] = temp[pivot_row][j];
                temp[pivot_row][j] = t;
            }
        }
        
        /* Eliminate */
        for (uint8_t row = col + 1; row < n; row++) {
            float factor = temp[row][col] / temp[col][col];
            for (uint8_t j = col; j < n; j++) {
                temp[row][j] -= factor * temp[col][j];
            }
        }
        
        determinant *= temp[col][col];
    }
    
    *det = sign * determinant;
    return MATRIX_OK;
}

int8_t matrixTrace(const Matrix_t *A, float *trace)
{
    if (A == NULL || trace == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    if (A->rows != A->cols) {
        return MATRIX_ERR_DIM_MISMATCH;
    }
    
    float sum = 0.0f;
    for (uint8_t i = 0; i < A->rows; i++) {
        sum += A->data[i * A->cols + i];
    }
    
    *trace = sum;
    return MATRIX_OK;
}

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

int8_t matrixCopy(const Matrix_t *src, Matrix_t *dest)
{
    if (src == NULL || dest == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    dest->rows = src->rows;
    dest->cols = src->cols;
    
    uint16_t total_elements = (uint16_t)src->rows * src->cols;
    memcpy(dest->data, src->data, total_elements * sizeof(float));
    
    return MATRIX_OK;
}

int8_t matrixEqual(const Matrix_t *A, const Matrix_t *B, uint8_t *equal)
{
    if (A == NULL || B == NULL || equal == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    *equal = 0;
    
    if (A->rows != B->rows || A->cols != B->cols) {
        return MATRIX_OK;
    }
    
    uint16_t total_elements = (uint16_t)A->rows * A->cols;
    for (uint16_t i = 0; i < total_elements; i++) {
        if (fabsf(A->data[i] - B->data[i]) > MATRIX_EPSILON) {
            return MATRIX_OK;
        }
    }
    
    *equal = 1;
    return MATRIX_OK;
}

int8_t matrixNorm(const Matrix_t *A, float *norm)
{
    if (A == NULL || norm == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    float sum_sq = 0.0f;
    uint16_t total_elements = (uint16_t)A->rows * A->cols;
    
    for (uint16_t i = 0; i < total_elements; i++) {
        sum_sq += A->data[i] * A->data[i];
    }
    
    *norm = sqrtf(sum_sq);
    return MATRIX_OK;
}

int8_t matrixLinspace(Matrix_t *mat, float start, float end, uint8_t n)
{
    if (mat == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    if (n == 0 || n > MATRIX_MAX_SIZE * MATRIX_MAX_SIZE) {
        return MATRIX_ERR_INVALID_SIZE;
    }
    
    mat->rows = 1;
    mat->cols = n;
    
    if (n == 1) {
        mat->data[0] = start;
        return MATRIX_OK;
    }
    
    float step = (end - start) / (float)(n - 1);
    for (uint8_t i = 0; i < n; i++) {
        mat->data[i] = start + step * (float)i;
    }
    
    return MATRIX_OK;
}

int8_t matrixLinIncrement(Matrix_t *mat, float start, float step, uint8_t n)
{
    if (mat == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    if (n == 0 || n > MATRIX_MAX_SIZE * MATRIX_MAX_SIZE) {
        return MATRIX_ERR_INVALID_SIZE;
    }
    
    mat->rows = 1;
    mat->cols = n;
    
    for (uint8_t i = 0; i < n; i++) {
        mat->data[i] = start + step * (float)i;
    }
    
    return MATRIX_OK;
}

int8_t matrixSubmatrix(const Matrix_t *A, Matrix_t *result,
                       uint8_t start_row, uint8_t start_col,
                       uint8_t num_rows, uint8_t num_cols)
{
    if (A == NULL || result == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    /* Check bounds */
    if (start_row + num_rows > A->rows || start_col + num_cols > A->cols) {
        return MATRIX_ERR_OUT_OF_BOUNDS;
    }
    
    if (num_rows > MATRIX_MAX_SIZE || num_cols > MATRIX_MAX_SIZE) {
        return MATRIX_ERR_INVALID_SIZE;
    }
    
    result->rows = num_rows;
    result->cols = num_cols;
    
    for (uint8_t i = 0; i < num_rows; i++) {
        for (uint8_t j = 0; j < num_cols; j++) {
            result->data[i * num_cols + j] = A->data[(start_row + i) * A->cols + (start_col + j)];
        }
    }
    
    return MATRIX_OK;
}

int8_t matrixSetSubmatrix(Matrix_t *A, const Matrix_t *sub,
                          uint8_t start_row, uint8_t start_col)
{
    if (A == NULL || sub == NULL) {
        return MATRIX_ERR_NULL_PTR;
    }
    
    /* Check bounds */
    if (start_row + sub->rows > A->rows || start_col + sub->cols > A->cols) {
        return MATRIX_ERR_OUT_OF_BOUNDS;
    }
    
    for (uint8_t i = 0; i < sub->rows; i++) {
        for (uint8_t j = 0; j < sub->cols; j++) {
            A->data[(start_row + i) * A->cols + (start_col + j)] = sub->data[i * sub->cols + j];
        }
    }
    
    return MATRIX_OK;
}
